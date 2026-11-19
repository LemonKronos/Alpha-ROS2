"""
The real trainer: runs the AI core Trainer (PPO + curriculum) on the Gazebo environment.

This file only connects existing pieces. It adds no learning logic:
    - GzEnvironment            (acrobatic_oa.environment)        the simulator side
    - SimulationRecovery       (acrobatic_oa.simulation_recovery) escalating restart ladder, as in ToyTrainer
    - Trainer / Curriculum / MonolithicPolicy (alpha_ai_core)   the learning side

How failures are handled:
    The AI core Trainer does not catch EnvironmentFailure, so it escapes from Trainer.train().
    Therefore training is run ONE PPO update at a time (train(total_updates=1)):
        - update finished            -> tell SimulationRecovery the simulation is healthy again.
        - EnvironmentFailure raised  -> SimulationRecovery.handle() runs the cheapest fitting recovery.
                                        The running episode is abandoned (the Trainer starts a new one) and the
                                        partly filled rollout is thrown away (the Trainer clears it at the start
                                        of the next rollout), so no transition of a broken episode is learned from.
        - recovery exhausted         -> the failure is re-raised, a checkpoint is saved, the node shuts down.

ROS 2 parameters (declared here, on the node):
    training_configuration_path : path to the TOML file, for example configs/hover.toml (required)
    resume_checkpoint_path      : path to a checkpoint to continue from, empty means start from scratch

Evaluation is intentionally not run here: load best.pt and evaluate it separately.
"""

import os
from pathlib import Path as PythonPath
import logging
import threading
import time
import traceback

import rclpy
from rclpy.node import Node

import torch

from alpha_ai_core.configuration import load_configuration
from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult
from alpha_ai_core.models.monolithic import MonolithicPolicy
from alpha_ai_core.models.spatial_cnn import ZeroSpatialEncoder
from alpha_ai_core.training.curriculum import Curriculum
from alpha_ai_core.training.environment import EnvironmentFailure
from alpha_ai_core.training.trainer import Trainer
from alpha_ai_core.utils.dashboard import TrainingDashboard

from acrobatic_oa.environment import GzEnvironment
from acrobatic_oa.simulation_recovery import SimulationRecovery

from python_utils.utils import *


PARAMETER_CONFIGURATION_PATH = "training_configuration_path"
PARAMETER_RESUME_CHECKPOINT_PATH = "resume_checkpoint_path"

#: Wall-clock seconds to let ROS discovery settle before the first reset (same as ToyTrainer)
STARTUP_DELAY_SECONDS = 1.0


class TrainingStopRequested(Exception):
    """
    Raised from inside the environment when the user asked to stop (Ctrl+C).
    It is deliberately NOT an EnvironmentFailure, so the recovery ladder never tries to "fix" it.
    """


class StoppableEnvironment:
    """
    Thin wrapper around GzEnvironment that implements the AI core Environment protocol.

    It adds two things the Trainer cannot do by itself:
        - It lets a stop request interrupt training in the middle of a rollout.
        - It records the wall-clock time of every reset and step.
    """
    def __init__(self, gz_environment: GzEnvironment, stop_event: threading.Event, analyzer: TimeAnalyzer):
        self.gz_environment = gz_environment
        self.stop_event = stop_event
        self.analyzer = analyzer

    def _raise_if_stop_requested(self) -> None:
        if self.stop_event.is_set():
            raise TrainingStopRequested()

    def reset(self, episode_configuration: EpisodeConfiguration) -> StepResult:
        self._raise_if_stop_requested()

        self.analyzer.start_segment("Reset")
        step_result = self.gz_environment.reset(episode_configuration)
        self.analyzer.stop_segment("Reset")
        return step_result

    def step(self, action: Action) -> StepResult:
        self._raise_if_stop_requested()

        self.analyzer.start_segment("Step")
        step_result = self.gz_environment.step(action)
        self.analyzer.stop_segment("Step")
        return step_result


class RealTrainer:
    def __init__(self, the_node: Node):
        self.the_node = the_node

        #: Make the AI core log lines visible (the Trainer logs through the standard logging module)
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

        self.stop_event = threading.Event()
        self.analyzer = TimeAnalyzer(the_node)

        #: Parameters
        configuration_path: str = Path.Dynamic.ACROBATIC_OA_AI_CONFIG().get()
        resume_checkpoint_path: str = Path.Dynamic.ACROBATIC_OA_RESUME_CHECKPOINT().get()

        #: Simulation side
        self.gz_environment = GzEnvironment(the_node)
        self.simulation_recovery = SimulationRecovery(the_node, self.gz_environment.world_handling)
        self.environment = StoppableEnvironment(self.gz_environment, self.stop_event, self.analyzer)

        #: Learning side. A strict configuration error stops here, before any simulation step.
        self.configuration = load_configuration(configuration_path)

        #: Seed BEFORE creating the policy, so the network initialization is repeatable
        torch.manual_seed(self.configuration.trainer.seed)
        policy = MonolithicPolicy(self.configuration.policy_network, self.configuration.observation)
        zero_cnn = ZeroSpatialEncoder(self.configuration.observation.spatial_feature_size)
        curriculum = Curriculum(self.configuration.curriculum, seed=self.configuration.trainer.seed)

        self.trainer = Trainer(
            policy=policy,
            environment=self.environment,
            curriculum=curriculum,
            configuration=self.configuration,
            spatial_encoder=zero_cnn,
            dashboard=TrainingDashboard(),
            log_file_name="acrobatic_oa_trainer")

        checkpoint_directory = PythonPath(os.path.expandvars(resume_checkpoint_path))
        if checkpoint_directory.is_file():
            self.trainer.load_checkpoint(resume_checkpoint_path)
            the_node.get_logger().info(
                f"Resumed from {resume_checkpoint_path} at update {self.trainer.update_count}"
            )


    def stop(self) -> None:
        self.stop_event.set()

    def training_loop(self) -> None:
        """Thread entry point. Any error that gets out of here is fatal: log it and shut the node down."""
        try:
            self._run_training()
        except Exception:
            self.the_node.get_logger().fatal(f"{RED} Real trainer stopped:\n{traceback.format_exc()} {RESET}")
            self.the_node.request_shutdown_after_failure()
            return

        if not self.stop_event.is_set():
            self.the_node.get_logger().info(f"{BLUE} Training finished: {self.trainer.update_count} updates {RESET}")
            rclpy.try_shutdown()

    def _run_training(self) -> None:
        time.sleep(STARTUP_DELAY_SECONDS)

        total_updates = self.configuration.trainer.total_updates
        update_count_at_start = self.trainer.update_count

        try:
            while self.trainer.update_count < total_updates and not self.stop_event.is_set():
                try:
                    self.trainer.train(total_updates=1)
                except EnvironmentFailure as failure:
                    self.the_node.get_logger().error(f"{RED} Environment failure: {failure} {RESET}")
                    if not self.simulation_recovery.handle(failure):
                        raise

                    #: The episode that was running is broken. Make the Trainer start a fresh one.
                    self.trainer.pending_inputs = None
                    continue

                self.simulation_recovery.report_success()
        except TrainingStopRequested:
            self.the_node.get_logger().warn(f"{YELLOW} Stop requested, training interrupted {RESET}")
        finally:
            self._save_progress(update_count_at_start)

    def _save_progress(self, update_count_at_start: int) -> None:
        """
        Saves last.pt (and best.pt when the score improved) so the work of this session is not lost.
        Nothing is saved when no update was finished in this session: that would overwrite an older
        checkpoint with an untrained policy.
        """
        if self.trainer.update_count <= update_count_at_start:
            return

        try:
            self.trainer.save_checkpoint()
        except Exception:
            self.the_node.get_logger().error(f"Could not save the final checkpoint:\n{traceback.format_exc()}")
