"""
Wrapper for the AI core environment
"""

from scipy.spatial.transform import Rotation

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from alpha_msgs.msg import ControlInterface, FusePerception, NormalizedMotorsServos, SpatialObservation, ContactSensor
from acrobatic_oa.world_handling import WorldHandling
from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult, TaskType, FloatArray, Vector3, Observation, SimulationInformation
from alpha_ai_core.training.environment import EnvironmentFailure
from python_utils.utils import *

class GzEnvironment:
    """
    This is the concrete class for the AI_core environment protocol
    """
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.world_handling = WorldHandling(the_node)

        self.body_contact = False
        self.propeller_contact = False

        #: Subscriber
        self.drone_contact = self.the_node.create_subscription(
            ContactSensor,
            Topic.CONTACT_PARSER,
            self.drone_contact_callback,
            qos_profile_sensor_data
        )


    def drone_contact_callback(self, msg: ContactSensor):
        self.body_contact = msg.bearable_contact
        self.propeller_contact = msg.critical_contact


    def _collect_step_result(self) -> StepResult:
        #: Collect Observation from sensor reading - Fuse Perception
        intent: FloatArray = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Hover only

        if self.the_node.latest_state is None:
            raise Exception("Unable to collect Observation")

        fuse_state: FusePerception = self.the_node.latest_state

        rotation = Rotation.from_quat(fuse_state.q, scalar_first=True)
        gravity_norm = np.asarray(-rotation.as_matrix()[2, :], dtype=np.float32)

        velocity_linear_norm = np.asarray(fuse_state.velocity, dtype=np.float32) / Physics.Drone.REF_LINEAR_VELOCITY
        velocity_angular_norm = np.asarray(fuse_state.angular_velocity, dtype=np.float32) / Physics.Drone.REF_ANGULAR_VELOCITY

        spatial_coarse = np.zeros(64, dtype=np.float32)
        spatial_detail = np.zeros(64, dtype=np.float32)

        #: Collect SimulationInformation from Ground Truth
        if self.world_handling.lastest_ground_truth is None:
            raise Exception("Unable to read ground truth")

        ground_truth = self.world_handling.lastest_ground_truth

        position: FloatArray = np.array(
            [
                ground_truth.pose.pose.position.x,
                ground_truth.pose.pose.position.y,
                ground_truth.pose.pose.position.z
            ],
            dtype=np.float32
        )

        simulation_time_message = self.world_handling.lastest_stats.sim_time
        simulation_time_seconds: float = Time.from_msg(simulation_time_message).nanoseconds * 1e-9

        #: Return
        return StepResult(
            Observation(
                state=np.concatenate((velocity_linear_norm, velocity_angular_norm, gravity_norm), dtype=np.float32),
                intent=intent,
                spatial_coarse=spatial_coarse,
                spatial_detail=spatial_detail
            ),
            SimulationInformation(
                position=position,
                body_contact=self.body_contact,
                propeller_contact=self.propeller_contact,
                simulation_time_seconds=simulation_time_seconds
            )
        )
    

    def reset(self, episode_config: EpisodeConfiguration) -> StepResult:
        #: Match case task type
        match episode_config.task:
            case TaskType.HOVER:
                self.world_handling.pause()
                spawn = episode_config.spawn

                #: Reset world
                self.world_handling.reset_all(episode_config.seed)

                #: Set spawn condition
                # Pose
                rotation = Rotation.from_euler('xyz', spawn.roll_pitch_yaw)
                qx, qy, qz, qw = rotation.as_quat()

                self.world_handling.teleport_drone(
                    spawn.position[0], spawn.position[1], spawn.position[2],
                    qw, qx, qy, qz
                )

                # Velocity -> force and torque
                rotation = rotation.as_matrix()
                inertia = rotation @ Drone.INERTIA @ rotation.T
                linear_velocity = np.array(spawn.linear_velocity)
                angular_velocity = np.array(spawn.angular_velocity)
                force = Drone.MASS * linear_velocity / Physics.STEP_SIZE
                torque = (inertia @ angular_velocity) / Physics.STEP_SIZE
                self.world_handling.set_drone_wrench(*force, *torque)

            case _:
                #TODO later 
                raise Exception(f"Task has not been implemented yet")

        #: Difficulty handling
        #TODO later 

        #! Step to get the first observation, this mean that the total time have to minus the first step. It OK because after step the Observation and Ground Truth does sync up 
        self.world_handling.step()
        time.sleep(0.01) #TODO replace with a "_wait_ready" with realtime 

        #: Return
        return self._collect_step_result()
        

    def step(self, action: Action) -> StepResult:
        #: Action step
        self.world_handling.pause()
        self.the_node.publish_action(action.values)
        self.world_handling.step()

        #: Wait for step completion
        #TODO look into self.world_handling.lastest_stats to check 

        #: Return
        return self._collect_step_result()
        




    