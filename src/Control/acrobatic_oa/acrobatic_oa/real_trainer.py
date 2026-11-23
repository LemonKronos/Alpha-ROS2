"""
The real trainer: runs the AI core Trainer (PPO + curriculum) on the Gazebo environment.

This file only connects existing pieces. It adds no learning logic and no recovery logic:
    - GzEnvironment, SimulationRecovery   (acrobatic_oa)  the simulator side
    - RecoveringEnvironment               (acrobatic_oa)  heals the simulator, talks to the AI core
    - Trainer, Curriculum, MonolithicPolicy (alpha_ai_core) the learning side

Simulation problems never reach this file:
    - The wrapper heals them. A lost episode becomes EpisodeAborted, which the Trainer handles itself.
    - Only when recovery is exhausted does EnvironmentFailure come out of Trainer.train().
      Then the progress of this session is saved and the node shuts down (training_loop).

Evaluation is intentionally not run here: load best.pt and evaluate it separately.
"""

import os
from pathlib import Path as PythonPath
import threading
import time
import traceback

import rclpy
from rclpy.node import Node

import torch

from alpha_ai_core.configuration import load_configuration
from alpha_ai_core.models.monolithic import MonolithicPolicy
from alpha_ai_core.models.spatial_cnn import ZeroSpatialEncoder
from alpha_ai_core.training.curriculum import Curriculum
from alpha_ai_core.training.trainer import Trainer
from alpha_ai_core.utils.dashboard_hook import DashboardHook

from acrobatic_oa.environment import GzEnvironment
from acrobatic_oa.recovering_environment import RecoveringEnvironment, TrainingStopRequested
from acrobatic_oa.simulation_recovery import SimulationRecovery

from python_utils.utils import *


#: Wall-clock seconds to let ROS discovery settle before the first reset (same as ToyTrainer)
STARTUP_DELAY_SECONDS = 1.0


class RealTrainer:
    def __init__(self, the_node: Node):
        self.the_node = the_node

        self.stop_event = threading.Event()
        self.analyzer = TimeAnalyzer(the_node)

        #: Parameters
        configuration_path: str = Path.Dynamic.ACROBATIC_OA_AI_CONFIG().get()
        resume_checkpoint_path: str = Path.Dynamic.ACROBATIC_OA_RESUME_CHECKPOINT().get()

        #: Simulation side
        self.gz_environment = GzEnvironment(the_node)
        self.simulation_recovery = SimulationRecovery(the_node, self.gz_environment.world_handling)
        self.environment = RecoveringEnvironment(
            the_node, self.gz_environment, self.simulation_recovery, self.stop_event, self.analyzer
        )

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
            dashboard_hook=DashboardHook(),
            log_file_name="acrobatic_oa_trainer")

        the_node.get_logger().info(f"Training device: {self.trainer.device}")

        checkpoint_file = PythonPath(os.path.expandvars(resume_checkpoint_path))
        if checkpoint_file.is_file():
            self.trainer.load_checkpoint(checkpoint_file)
            the_node.get_logger().warn(
                f"Resumed from {checkpoint_file} at update {self.trainer.update_count}"
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
            self.the_node.get_logger().warn(f"{GREEN} Training finished: {self.trainer.update_count} updates {RESET}")
            rclpy.try_shutdown()

    def _run_training(self) -> None:
        time.sleep(STARTUP_DELAY_SECONDS)

        update_count_at_start = self.trainer.update_count
        remaining_updates = self.configuration.trainer.total_updates - update_count_at_start
        if remaining_updates <= 0:
            self.the_node.get_logger().warn(
                f"{RED} The checkpoint already has {update_count_at_start} updates, nothing left to train {RESET}"
            )
            return

        try:
            #: One call. Simulation problems are handled below this line (see the module docstring).
            self.trainer.train(total_updates=remaining_updates)
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