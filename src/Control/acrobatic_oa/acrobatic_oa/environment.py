"""
Wrapper for the AI core environment
"""

import threading
import time
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from nav_msgs.msg import Odometry
from alpha_msgs.msg import FusePerception, ContactSensor
from acrobatic_oa.world_handling import WorldHandling
from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult, TaskType, FloatArray, Observation, SimulationInformation
from alpha_ai_core.training.environment import EnvironmentFailure
from python_utils.utils import *

#: Timing
CONTROL_STEP_NANOSECONDS = round(Clock.LOOP_CYCLE_FAST_NANOSEC)
STAMP_TOLERANCE_NANOSECONDS = CONTROL_STEP_NANOSECONDS // 2

#: Waiting policy, timeouts are WALL-CLOCK seconds (the simulation is paused while we wait)
STEP_WAIT_TIMEOUT_SECONDS = 0.1
RESET_WAIT_TIMEOUT_SECONDS = 1.0
MAXIMUM_CONSECUTIVE_MISSES_PER_STEP = 2
MAXIMUM_CONSECUTIVE_MISSES_AT_RESET = 0
RESET_MAXIMUM_ATTEMPTS = 3
PIPELINE_CONNECTION_TIMEOUT_SECONDS = 2.0

ACTION_VALUE_COUNT = 8

SOURCE_FUSE_PERCEPTION = "fuse_perception"
SOURCE_GROUND_TRUTH = "ground_truth"


def stamp_to_nanoseconds(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass(frozen=True)
class TimedMessage:
    message: object
    stamp_nanoseconds: int


class WorldServiceFailure(EnvironmentFailure):
    """A Gazebo world call failed or timed out: Gazebo is down, hung or unreachable."""

class ObservationTimeoutFailure(EnvironmentFailure):
    """World calls work but the pipeline did not deliver the observation: a pipeline node is down or stuck."""

class ObservationInvalidFailure(EnvironmentFailure):
    """The observation holds NaN or infinity: the physics most likely diverged. A plain reset fixes it."""

class PipelineNodeMissingFailure(EnvironmentFailure):
    """A node the episode depends on is not connected, so the episode would silently be wrong."""

class ObservationSnapshot:
    """
    Thread-safe box between the ROS callbacks (spin thread) and the training thread.

    Keeps the latest message of every required source. The training thread blocks until every
    source holds a message stamped for the step we just took, then reads them all at once.
    """
    def __init__(self, source_names: list[str]):
        self.source_names = list(source_names)
        self.condition = threading.Condition()
        self.latest_messages: dict[str, TimedMessage | None] = {name: None for name in self.source_names}
        self.consecutive_misses: dict[str, int] = {name: 0 for name in self.source_names}

    def store(self, source_name: str, message) -> None:
        timed_message = TimedMessage(message, stamp_to_nanoseconds(message.header.stamp))
        with self.condition:
            self.latest_messages[source_name] = timed_message
            self.condition.notify_all()

    def clear(self) -> None:
        with self.condition:
            for source_name in self.source_names:
                self.latest_messages[source_name] = None
                self.consecutive_misses[source_name] = 0

    def _is_matching(self, source_name: str, expected_stamp_nanoseconds: int) -> bool:
        """The caller must hold the condition lock."""
        timed_message = self.latest_messages[source_name]
        if timed_message is None:
            return False
        stamp_difference = abs(timed_message.stamp_nanoseconds - expected_stamp_nanoseconds)
        return stamp_difference <= STAMP_TOLERANCE_NANOSECONDS

    def wait_and_collect(
        self,
        expected_stamp_nanoseconds: int,
        timeout_seconds: float,
        maximum_consecutive_misses: int
    ) -> tuple[dict[str, TimedMessage], list[str]]:
        """
        Wait (wall clock) until every source has a message stamped for the expected simulation time.

        A source that did not arrive in time is a "miss": its previous message is reused, unless it
        has missed more than maximum_consecutive_misses times in a row or has no previous message.
        Raises ObservationTimeoutFailure in that case.

        Returns the collected messages and the names of the sources that were reused (missed).
        """
        with self.condition:
            self.condition.wait_for(
                lambda: all(self._is_matching(name, expected_stamp_nanoseconds) for name in self.source_names),
                timeout_seconds
            )

            collected_messages: dict[str, TimedMessage] = {}
            missed_source_names: list[str] = []
            failure_descriptions: list[str] = []

            for source_name in self.source_names:
                timed_message = self.latest_messages[source_name]

                if self._is_matching(source_name, expected_stamp_nanoseconds):
                    self.consecutive_misses[source_name] = 0
                    collected_messages[source_name] = timed_message
                    continue

                self.consecutive_misses[source_name] += 1
                missed_source_names.append(source_name)

                too_many_misses = self.consecutive_misses[source_name] > maximum_consecutive_misses
                if too_many_misses or timed_message is None:
                    latest_stamp = "none" if timed_message is None else timed_message.stamp_nanoseconds
                    failure_descriptions.append(
                        f"{source_name}: expected stamp {expected_stamp_nanoseconds} ns, "
                        f"latest stamp {latest_stamp} ns, "
                        f"consecutive misses {self.consecutive_misses[source_name]}"
                    )
                else:
                    collected_messages[source_name] = timed_message

            if failure_descriptions:
                raise ObservationTimeoutFailure("Observation not ready: " + "; ".join(failure_descriptions))

            return collected_messages, missed_source_names

class GzEnvironment:
    """
    This is the concrete class for the AI_core environment protocol
    """
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.world_handling = WorldHandling(the_node)

        self.body_contact = False
        self.propeller_contact = False

        #: Simulation time we expect the next observation to be stamped with (nanoseconds since reset)
        self.expected_stamp_nanoseconds = 0

        self.observation_snapshot = ObservationSnapshot([SOURCE_FUSE_PERCEPTION, SOURCE_GROUND_TRUTH])

        #: Subscribers
        self.SUB_fuse_perception = self.the_node.create_subscription(
            FusePerception,
            Topic.FUSE_PERCEPTION,
            lambda message: self.observation_snapshot.store(SOURCE_FUSE_PERCEPTION, message),
            qos_profile_sensor_data
        )

        self.SUB_ground_truth = self.the_node.create_subscription(
            Odometry,
            Topic.DRONE_ODOMETRY,
            lambda message: self.observation_snapshot.store(SOURCE_GROUND_TRUTH, message),
            qos_profile_system_default
        )

        self.drone_contact = self.the_node.create_subscription(
            ContactSensor,
            Topic.CONTACT_PARSER,
            self.drone_contact_callback,
            qos_profile_sensor_data
        )


    def drone_contact_callback(self, msg: ContactSensor):
        self.body_contact = msg.bearable_contact
        self.propeller_contact = msg.critical_contact


    @staticmethod
    def _require(succeeded: bool, description: str) -> None:
        if not succeeded:
            raise WorldServiceFailure(f"Failed to {description}")


    def _collect_step_result(self, timeout_seconds: float, maximum_consecutive_misses: int) -> StepResult:
        timed_messages, missed_source_names = self.observation_snapshot.wait_and_collect(
            self.expected_stamp_nanoseconds, timeout_seconds, maximum_consecutive_misses
        )
        if missed_source_names:
            self.the_node.get_logger().warn(f"{YELLOW} Reused previous message of: {missed_source_names} {RESET}")

        fuse_state: FusePerception = timed_messages[SOURCE_FUSE_PERCEPTION].message
        ground_truth: Odometry = timed_messages[SOURCE_GROUND_TRUTH].message

        #: Observation from Fuse Perception
        intent: FloatArray = np.zeros(6, dtype=np.float32) #DEV Hover only

        rotation = Rotation.from_quat(fuse_state.q, scalar_first=True)
        gravity_norm = np.asarray(-rotation.as_matrix()[2, :], dtype=np.float32)

        velocity_linear_norm = np.asarray(fuse_state.velocity, dtype=np.float32) / Physics.Drone.REF_LINEAR_VELOCITY
        velocity_angular_norm = np.asarray(fuse_state.angular_velocity, dtype=np.float32) / Physics.Drone.REF_ANGULAR_VELOCITY

        spatial_coarse = np.zeros(64, dtype=np.float32)
        spatial_detail = np.zeros(64, dtype=np.float32)

        #: Simulation information from Ground Truth
        position: FloatArray = np.array(
            [
                ground_truth.pose.pose.position.x,
                ground_truth.pose.pose.position.y,
                ground_truth.pose.pose.position.z
            ],
            dtype=np.float32
        )

        #: Exact by construction, and unaffected by a tolerated missed message
        simulation_time_us: int = self.expected_stamp_nanoseconds // 1_000

        #: NaN check states
        state = np.concatenate((velocity_linear_norm, velocity_angular_norm, gravity_norm), dtype=np.float32)
        if not (np.all(np.isfinite(state)) and np.all(np.isfinite(position))):
            raise ObservationInvalidFailure(f"Observation is not finite: state={state}, position={position}")

        return StepResult(
            Observation(
                state=state,
                intent=intent,
                spatial_coarse=spatial_coarse,
                spatial_detail=spatial_detail
            ),
            SimulationInformation(
                position=position,
                body_contact=self.body_contact,
                propeller_contact=self.propeller_contact,
                simulation_time_us=simulation_time_us
            )
        )
    

    def reset(self, episode_config: EpisodeConfiguration) -> StepResult:
        if episode_config.task != TaskType.HOVER:
            #TODO later
            raise NotImplementedError("Task has not been implemented yet")

        last_failure: EnvironmentFailure | None = None
        for attempt_number in range(1, RESET_MAXIMUM_ATTEMPTS + 1):
            try:
                return self._attempt_reset(episode_config)
            except EnvironmentFailure as failure:
                last_failure = failure
                self.the_node.get_logger().warn(
                    f"{YELLOW} Reset attempt {attempt_number}/{RESET_MAXIMUM_ATTEMPTS} failed: {failure} {RESET}"
                )

        #TODO later: this is where a Gazebo/pipeline restart can hook in
        self.the_node.get_logger().error(f"Reset failed {RESET_MAXIMUM_ATTEMPTS} times")
        raise last_failure


    def _require_pipeline_connected(self) -> None:
        """
        Fail loudly when nobody receives our actions or nobody publishes contacts.
        Waits a short wall-clock time first, because discovery lags right after a pipeline restart.
        """
        deadline = time.monotonic() + PIPELINE_CONNECTION_TIMEOUT_SECONDS
        while True:
            if self.the_node.PUB_acrobatic.get_subscription_count() == 0:
                missing_description = "nobody subscribes to the action topic (finalize_control down?)"
            elif self.drone_contact.get_publisher_count() == 0:
                missing_description = "nobody publishes contacts (contact_parser down?)"
            else:
                return

            if time.monotonic() >= deadline:
                raise PipelineNodeMissingFailure(f"Pipeline not connected: {missing_description}")
            time.sleep(0.05)


    def _attempt_reset(self, episode_config: EpisodeConfiguration) -> StepResult:
        #: Ensure the pipeline is running
        self._require_pipeline_connected()

        spawn = episode_config.spawn

        #: Reset world
        self._require(self.world_handling.pause(), "pause the world")
        self.observation_snapshot.clear()
        self._require(self.world_handling.reset_all(episode_config.seed), "reset the world")
        self.expected_stamp_nanoseconds = 0

        #: Neutral actuators, so no command from the previous episode is acting
        self.the_node.publish_action(np.zeros(ACTION_VALUE_COUNT, dtype=np.float32))

        #: Set spawn condition
        # Pose
        rotation = Rotation.from_euler('xyz', spawn.roll_pitch_yaw)
        qx, qy, qz, qw = rotation.as_quat()

        self._require(
            self.world_handling.teleport_drone(
                spawn.position[0], spawn.position[1], spawn.position[2],
                qw, qx, qy, qz
            ),
            "teleport the drone"
        )

        # Velocity -> force and torque
        rotation_matrix = rotation.as_matrix()
        inertia = rotation_matrix @ Drone.INERTIA @ rotation_matrix.T
        linear_velocity = np.array(spawn.linear_velocity)
        angular_velocity = np.array(spawn.angular_velocity)
        force = Drone.MASS * linear_velocity / Physics.STEP_SIZE
        torque = (inertia @ angular_velocity) / Physics.STEP_SIZE
        self._require(self.world_handling.set_drone_wrench(*force, *torque), "apply the spawn wrench")

        #: Difficulty handling
        #TODO later

        #! Step to get the first observation, the total time has to minus this first step.
        self._require(self.world_handling.step(), "step the world after reset")
        self.expected_stamp_nanoseconds += CONTROL_STEP_NANOSECONDS

        return self._collect_step_result(RESET_WAIT_TIMEOUT_SECONDS, MAXIMUM_CONSECUTIVE_MISSES_AT_RESET)
        

    def step(self, action: Action) -> StepResult:
        #: Ensure the pipeline is running
        self._require_pipeline_connected()

        #: Step action
        self.the_node.publish_action(action.values)
        self._require(self.world_handling.step(), "step the world")
        self.expected_stamp_nanoseconds += CONTROL_STEP_NANOSECONDS

        return self._collect_step_result(STEP_WAIT_TIMEOUT_SECONDS, MAXIMUM_CONSECUTIVE_MISSES_PER_STEP)
        




    