"""
Wrapper for the AI core environment
"""

from scipy.spatial.transform import Rotation

from rclpy.node import Node
from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult, TaskType, FloatArray, Vector3
from alpha_ai_core.training.environment import EnvironmentFailure
from acrobatic_oa.world_handling import WorldHandling
from python_utils.utils import *

class GzEnvironment:
    """
    This is the concrete class for the AI_core environment protocol
    """
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.world_handling = WorldHandling(the_node)

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

                #DEV test play
                self.world_handling.play()

            case _:
                pass

        #: Difficulty handling

        #: Return
        # return StepResult()
        

    def step(self, action: Action) -> StepResult:
        #: Pause
        # self.world_handling.pause()

        #: Queue action
        self.the_node.publish_action(action.values)

        #: Step
        # self.world_handling.step()

        #: Return
        # return StepResult()




    