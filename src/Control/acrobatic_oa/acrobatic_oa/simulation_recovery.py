"""
Escalating recovery from simulation failures, shared by the toy trainer and the real trainer.

Ladder:
    1. Retry the reset            (done inside GzEnvironment.reset)
    2. Restart Gazebo             -> restartGazebo.sh
    3. Restart the pipeline       -> restartPipeline.sh
    4. Give up: handle() returns False, the caller logs and shuts down. No reboot.
"""
import os
import subprocess
from enum import Enum

import rclpy
from rclpy.node import Node

from alpha_ai_core.interface import EnvironmentFailure
from acrobatic_oa.environment import WorldServiceFailure, ObservationInvalidFailure
from acrobatic_oa.world_handling import WorldHandling
from python_utils.utils import *

SCRIPT_RESTART_GAZEBO = "restartGazebo.sh"
SCRIPT_RESTART_PIPELINE = "restartPipeline.sh"
SCRIPT_LOG_FOLDER = "/home/mr_lemon/.ros/log"

RESTART_GAZEBO_TIMEOUT_SECONDS = 120.0
RESTART_PIPELINE_TIMEOUT_SECONDS = 120.0
WORLD_SERVICES_TIMEOUT_SECONDS = 60.0

MAXIMUM_ATTEMPTS_PER_LEVEL = 2
MAXIMUM_CONSECUTIVE_INVALID_OBSERVATIONS = 5


class RecoveryLevel(Enum):
    RESET_ONLY = "reset only"
    RESTART_GAZEBO = "restart Gazebo"
    RESTART_PIPELINE = "restart pipeline"


class SimulationRecovery:
    def __init__(self, the_node: Node, world_handling: WorldHandling):
        self.the_node = the_node
        self.world_handling = world_handling

        scripts_folder = Path.Dynamic.SIMULATION_SCRIPTS().get()
        self.restart_gazebo_script = os.path.join(scripts_folder, SCRIPT_RESTART_GAZEBO)
        self.restart_pipeline_script = os.path.join(scripts_folder, SCRIPT_RESTART_PIPELINE)

        #: Fail at start-up, not in the middle of a failure
        for script_path in (self.restart_gazebo_script, self.restart_pipeline_script):
            if not os.path.isfile(script_path):
                raise FileNotFoundError(
                    f"Recovery script not found: {script_path} (is SIMULATION_SCRIPTS_PATH exported?)"
                )

        self.consecutive_gazebo_restarts = 0
        self.consecutive_pipeline_restarts = 0
        self.consecutive_invalid_observations = 0

    def report_success(self) -> None:
        """Call after a completed episode: the simulation is healthy again."""
        self.consecutive_gazebo_restarts = 0
        self.consecutive_pipeline_restarts = 0
        self.consecutive_invalid_observations = 0

    def handle(self, failure: EnvironmentFailure) -> bool:
        """
        Run the cheapest recovery that fits the failure.
        Returns True when the caller should try again, False when recovery is exhausted.
        """
        if not rclpy.ok():
            #: Shutting down: a restart script now would only block the exit
            return False
        
        level = self._choose_level(failure)
        if level is None:
            self.the_node.get_logger().fatal(f"{RED} Recovery exhausted, last failure: {failure} {RESET}")
            return False

        self.the_node.get_logger().warn(f"{YELLOW} Recovery: {level.value} (failure: {failure}) {RESET}")

        match level:
            case RecoveryLevel.RESET_ONLY:
                pass
            case RecoveryLevel.RESTART_GAZEBO:
                self.consecutive_gazebo_restarts += 1
                self.consecutive_invalid_observations = 0
                self._restart_gazebo()
            case RecoveryLevel.RESTART_PIPELINE:
                self.consecutive_pipeline_restarts += 1
                self._restart_pipeline()

        return True

    def _choose_level(self, failure: EnvironmentFailure) -> RecoveryLevel | None:
        if isinstance(failure, ObservationInvalidFailure):
            self.consecutive_invalid_observations += 1
            if self.consecutive_invalid_observations <= MAXIMUM_CONSECUTIVE_INVALID_OBSERVATIONS:
                return RecoveryLevel.RESET_ONLY
            wanted_level = RecoveryLevel.RESTART_GAZEBO # keeps diverging, treat it as a Gazebo problem
        elif isinstance(failure, WorldServiceFailure):
            wanted_level = RecoveryLevel.RESTART_GAZEBO
        else:
            wanted_level = RecoveryLevel.RESTART_PIPELINE

        if wanted_level == RecoveryLevel.RESTART_GAZEBO and self.consecutive_gazebo_restarts >= MAXIMUM_ATTEMPTS_PER_LEVEL:
            wanted_level = RecoveryLevel.RESTART_PIPELINE
        if wanted_level == RecoveryLevel.RESTART_PIPELINE and self.consecutive_pipeline_restarts >= MAXIMUM_ATTEMPTS_PER_LEVEL:
            return None
        return wanted_level

    def _restart_gazebo(self) -> bool:
        if not self._run_script(self.restart_gazebo_script, RESTART_GAZEBO_TIMEOUT_SECONDS):
            return False
        services_ready = self.world_handling.wait_for_services(WORLD_SERVICES_TIMEOUT_SECONDS)
        if not services_ready:
            self.the_node.get_logger().error("World services did not come back after the Gazebo restart")
        return services_ready

    def _restart_pipeline(self) -> bool:
        return self._run_script(self.restart_pipeline_script, RESTART_PIPELINE_TIMEOUT_SECONDS)

    def _run_script(self, script_path: str, timeout_seconds: float) -> bool:
        log_path = os.path.join(SCRIPT_LOG_FOLDER, f"{os.path.basename(script_path)}.log")
        try:
            with open(log_path, "w") as log_file:
                completed_process = subprocess.run(
                    ["bash", script_path],
                    stdin=subprocess.DEVNULL,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    timeout=timeout_seconds,
                    start_new_session=True
                )
        except subprocess.TimeoutExpired:
            self.the_node.get_logger().error(f"{script_path} timed out after {timeout_seconds}s, see {log_path}")
            return False

        if completed_process.returncode != 0:
            self.the_node.get_logger().error(
                f"{script_path} exited with {completed_process.returncode}, see {log_path}"
            )
            return False
        return True