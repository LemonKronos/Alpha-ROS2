"""
The environment the AI core Trainer talks to during real training.

Job split:
    GzEnvironment         one attempt at everything. Raises EnvironmentFailure subclasses when something breaks.
    SimulationRecovery    knows HOW to heal (retry, restart Gazebo, restart pipeline).
    RecoveringEnvironment (this file) decides WHEN to heal, and translates the outcome for the AI core:

        reset() fails   -> heal and try again, inside the call. The AI core never notices.
        step()  fails   -> heal, then raise EpisodeAborted: the world of the episode is gone, but the
                           environment is healthy again. The AI core ends that episode and calls reset().
        recovery exhausted -> raise EnvironmentFailure. The AI core does not catch it: it stops training.

It also lets a stop request (Ctrl+C) interrupt training in the middle of a rollout, and records the
wall-clock time of every reset and step.
"""

import threading

from rclpy.node import Node

from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult
from alpha_ai_core.training.environment import EnvironmentFailure, EpisodeAborted

from acrobatic_oa.environment import GzEnvironment
from acrobatic_oa.simulation_recovery import SimulationRecovery

from python_utils.utils import *



class TrainingStopRequested(Exception):
    """
    Raised from inside the environment when the user asked to stop (Ctrl+C).
    It is deliberately NOT an EnvironmentFailure or an EpisodeAborted, so neither the recovery
    nor the AI core ever treats it as a simulation problem.
    """


class RecoveringEnvironment:
    """Implements the AI core Environment protocol on top of GzEnvironment."""

    def __init__(
        self,
        the_node: Node,
        gz_environment: GzEnvironment,
        simulation_recovery: SimulationRecovery,
        stop_event: threading.Event,
        analyzer: TimeAnalyzer
    ):
        self.the_node = the_node
        self.gz_environment = gz_environment
        self.simulation_recovery = simulation_recovery
        self.stop_event = stop_event
        self.analyzer = analyzer

        self.steps_in_current_episode = 0
        self.current_episode_was_aborted = False

    def _raise_if_stop_requested(self) -> None:
        if self.stop_event.is_set():
            raise TrainingStopRequested()

    def _log_failure(self, failure: EnvironmentFailure) -> None:
        self.the_node.get_logger().error(f"{RED} Environment failure: {failure} {RESET}")

    def reset(self, episode_configuration: EpisodeConfiguration) -> StepResult:
        """Heals and retries until the episode starts, or recovery is exhausted (EnvironmentFailure)."""

        if self.steps_in_current_episode > 0 and not self.current_episode_was_aborted:
            # The previous episode ended by itself (success, contact or timeout): the simulation is healthy
            self.simulation_recovery.report_success()
        self.steps_in_current_episode = 0
        self.current_episode_was_aborted = False

        while True:
            self._raise_if_stop_requested()

            self.analyzer.start_segment("Reset")
            try:
                step_result = self.gz_environment.reset(episode_configuration)
            except EnvironmentFailure as failure:
                self._log_failure(failure)
                if not self.simulation_recovery.handle(failure):
                    raise
                continue

            self.analyzer.stop_segment("Reset")
            return step_result

    def step(self, action: Action) -> StepResult:
        """
        Heals after a failure, then raises EpisodeAborted: the simulation is healthy again,
        but the episode that was running is lost.
        """
        self._raise_if_stop_requested()

        self.analyzer.start_segment("Step")
        try:
            step_result = self.gz_environment.step(action)
        except EnvironmentFailure as failure:
            self.current_episode_was_aborted = True
            self._log_failure(failure)
            if not self.simulation_recovery.handle(failure):
                raise
            raise EpisodeAborted(f"The simulation was healed after: {failure}") from failure

        self.analyzer.stop_segment("Step")

        self.steps_in_current_episode += 1
        return step_result