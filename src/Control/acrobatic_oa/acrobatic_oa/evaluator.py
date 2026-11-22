"""
The evaluator: evaluates the untrained baseline (optional), THEN a checkpoint, on the Gazebo environment.
No learning happens.

It has the same interface as RealTrainer and ToyTrainer (training_loop, stop, analyzer), so the node can
use it in the same place:  self.trainer = Evaluator(self)

What it does:
    1. Builds the same policy, curriculum and Trainer as RealTrainer (the Trainer is only used for its
       evaluate(): no exploration noise, the deployment path, fixed evaluation seed from the TOML).
       Two Trainers are built: one for the checkpoint, and (if BASELINE_EVALUATION) one that is never
       loaded, so it keeps the untrained weights.
    2. Loads the checkpoint into its Trainer and reads the difficulty the checkpoint had reached.
    3. Runs Trainer.evaluate() on the untrained baseline, then on the checkpoint, both at that difficulty.
    4. Logs both results and the difference, and shuts the node down.

Simulation problems are handled exactly as in training (RecoveringEnvironment): an episode the
simulator lost is run again from the start, so every result comes from a complete episode.

The baseline starts from the same random weights as a training run, because the same seed is set before
the policy is created.
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
from alpha_ai_core.models.monolithic import MonolithicPolicy
from alpha_ai_core.models.spatial_cnn import ZeroSpatialEncoder
from alpha_ai_core.training.curriculum import Curriculum
from alpha_ai_core.training.trainer import Trainer

from acrobatic_oa.environment import GzEnvironment
from acrobatic_oa.recovering_environment import RecoveringEnvironment, TrainingStopRequested
from acrobatic_oa.simulation_recovery import SimulationRecovery

from python_utils.utils import *


# The checkpoint to evaluate, for example best.pt. Environment variables are expanded. Mandatory:
# a missing path or a missing file is an error.
EVALUATION_CHECKPOINT_PATH: str | None = Path.Dynamic.ACROBATIC_OA_EVALUATION_CHECKPOINT().get()

# Evaluate the untrained baseline before the checkpoint? Set to False to skip it.
BASELINE_EVALUATION: bool = True

# Optional override of the difficulty, for example {"spawn": 1.0}. The dimensions that are not listed keep the
# value the checkpoint had reached. None means: no override, use the checkpoint's difficulty as it is.
# Either way, the baseline and the checkpoint are evaluated at the same, explicit difficulty.
EVALUATION_DIFFICULTY_OVERRIDE: dict[str, float] | None = None

# The TOML the checkpoint was trained with (it must be compatible, the checkpoint is checked on loading)
CONFIGURATION_PATH: str = Path.Dynamic.ACROBATIC_OA_AI_CONFIG().get()

# Wall-clock seconds to let ROS discovery settle before the first reset (same as the trainers)
STARTUP_DELAY_SECONDS = 1.0


class Evaluator:
    def __init__(self, the_node: Node):
        self.the_node = the_node

        # Make the AI core log lines visible (the Trainer logs through the standard logging module)
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

        self.stop_event = threading.Event()

        # Fail before anything is built if the checkpoint is not usable
        self.checkpoint_file = self._resolve_checkpoint_file()

        # Simulation side
        self.gz_environment = GzEnvironment(the_node)
        self.simulation_recovery = SimulationRecovery(the_node, self.gz_environment.world_handling)
        self.environment = RecoveringEnvironment(
            the_node, self.gz_environment, self.simulation_recovery, self.stop_event
        )

        # Learning side. A strict configuration error stops here, before any simulation step.
        self.configuration = load_configuration(CONFIGURATION_PATH)

        # The checkpoint Trainer: loaded now, because its difficulty is needed for both evaluations
        self.checkpoint_trainer = self._build_trainer("acrobatic_oa_evaluator_checkpoint")
        the_node.get_logger().info(f"Evaluation device: {self.checkpoint_trainer.device}")

        self.checkpoint_trainer.load_checkpoint(self.checkpoint_file)
        the_node.get_logger().warn(
            f"Loaded {self.checkpoint_file} (trained for {self.checkpoint_trainer.update_count} updates)"
        )

        # The one difficulty used for every evaluation (a copy: nothing can change it afterwards)
        self.difficulty: dict[str, float] = dict(self.checkpoint_trainer.curriculum.difficulty)
        if EVALUATION_DIFFICULTY_OVERRIDE is not None:
            self.difficulty.update(EVALUATION_DIFFICULTY_OVERRIDE)
        the_node.get_logger().warn(f"Evaluation difficulty (baseline and checkpoint): {self.difficulty}")

        # The baseline Trainer is never loaded: it keeps the untrained weights
        self.baseline_trainer: Trainer | None = None
        if BASELINE_EVALUATION:
            self.baseline_trainer = self._build_trainer("acrobatic_oa_evaluator_baseline")
        else:
            the_node.get_logger().warn(f"{YELLOW} BASELINE_EVALUATION is False: the baseline is skipped {RESET}")

    def _resolve_checkpoint_file(self) -> PythonPath:
        if EVALUATION_CHECKPOINT_PATH is None:
            raise ValueError("EVALUATION_CHECKPOINT_PATH is None: a checkpoint is required to evaluate")

        checkpoint_file = PythonPath(os.path.expandvars(EVALUATION_CHECKPOINT_PATH))
        if not checkpoint_file.is_file():
            raise FileNotFoundError(f"Checkpoint to evaluate not found: {checkpoint_file}")
        return checkpoint_file

    def _build_trainer(self, log_file_name: str) -> Trainer:
        # Same seed as training, so an untouched Trainer holds the policy a training run starts from
        torch.manual_seed(self.configuration.trainer.seed)
        policy = MonolithicPolicy(self.configuration.policy_network, self.configuration.observation)
        zero_cnn = ZeroSpatialEncoder(self.configuration.observation.spatial_feature_size)
        curriculum = Curriculum(self.configuration.curriculum, seed=self.configuration.trainer.seed)

        return Trainer(
            policy=policy,
            environment=self.environment,
            curriculum=curriculum,
            configuration=self.configuration,
            spatial_encoder=zero_cnn,
            dashboard_hook=None,
            log_file_name=log_file_name)

    def stop(self) -> None:
        self.stop_event.set()

    def training_loop(self) -> None:
        """Thread entry point (the name matches the trainers). Any error that gets out of here is fatal."""
        try:
            time.sleep(STARTUP_DELAY_SECONDS)

            baseline_results = None
            if self.baseline_trainer is not None:
                self.the_node.get_logger().warn(f"{YELLOW} Evaluating the UNTRAINED baseline {RESET}")
                baseline_results = self._run_evaluation(self.baseline_trainer)
                self._log_results("Baseline (untrained)", baseline_results)

            self.the_node.get_logger().warn(f"{YELLOW} Evaluating the checkpoint {RESET}")
            checkpoint_results = self._run_evaluation(self.checkpoint_trainer)
            self._log_results(f"Checkpoint ({self.checkpoint_file.name})", checkpoint_results)
        except TrainingStopRequested:
            self.the_node.get_logger().warn(f"{YELLOW} Stop requested, evaluation interrupted {RESET}")
            return
        except Exception:
            self.the_node.get_logger().fatal(f"{RED} Evaluator stopped:\n{traceback.format_exc()} {RESET}")
            self.the_node.request_shutdown_after_failure()
            return

        if baseline_results is not None:
            self._log_comparison(baseline_results, checkpoint_results)
        rclpy.try_shutdown()

    def _run_evaluation(self, trainer: Trainer) -> dict:
        # The number of episodes and the seed come from the TOML (evaluation_episodes, evaluation_seed)
        # Always an explicit difficulty, and a fresh copy each time
        return trainer.evaluate(difficulty=dict(self.difficulty))

    def _log_results(self, title: str, results: dict) -> None:
        lines = [f"{GREEN} Evaluation finished: {title} {RESET}"]
        for name, value in results.items():
            lines.append(f"    {name}: {value:.4f}" if isinstance(value, float) else f"    {name}: {value}")
        self.the_node.get_logger().warn("\n".join(lines))

    def _log_comparison(self, baseline: dict, checkpoint: dict) -> None:
        def is_number(x) -> bool:
            return isinstance(x, (int, float)) and not isinstance(x, bool)

        lines = [f"{GREEN} Checkpoint vs baseline (same difficulty) {RESET}"]
        for name, base_value in baseline.items():
            value = checkpoint.get(name)
            if is_number(base_value) and is_number(value):
                lines.append(f"    {name}: {base_value:.4f} -> {value:.4f} ({value - base_value:+.4f})")
        self.the_node.get_logger().warn("\n".join(lines))