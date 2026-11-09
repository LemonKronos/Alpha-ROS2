"""
Wrapper for the AI core environment
"""

from scipy.spatial.transform import Rotation

from rclpy.node import Node
from alpha_ai_core.interface import Action, EpisodeConfiguration, StepResult, TaskType
from acrobatic_oa.world_handling import WorldHandling
from python_utils.utils import *

class GzEnvironment:
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.world_handling = WorldHandling()

    def reset(self, episode_config: EpisodeConfiguration) -> StepResult:
        # Ignore the seed reset with seed for now.

        match episode_config.task:
            case TaskType.HOVER:
                self.world_handling.pause()

                # Reset
                self.world_handling.teleport_drone(
                    0.0, 0.0, 0.25,
                    1.0, 0.0, 0.0, 0.0
                )
                self.world_handling.zero_drone_speed()
                #TODO change these teleport and zero to just reset drone 

                spawn = episode_config.spawn

                # Pose
                rotation = Rotation.from_euler('xyz', spawn.roll_pitch_yaw)
                qx, qy, qz, qw = rotation.as_quat()

                self.world_handling.teleport_drone(
                    spawn.position[0], spawn.position[1], spawn.position[2],
                    qw, qx, qy, qz
                )

                # Velocity
                rotation = rotation.as_matrix()
                inertia = rotation @ Drone.INERTIA @ rotation.T
                linear_velocity = np.array(spawn.linear_velocity)
                angular_velocity = np.array(spawn.angular_velocity)
                force = Drone.MASS * linear_velocity / Physics.STEP_SIZE
                torque = (inertia @ angular_velocity) / Physics.STEP_SIZE
                self.world_handling.set_drone_wrench(*force, *torque)

            case _:
                pass

            # Difficulty handling

        

    def step(self, action: Action) -> StepResult:
        # Queue action

        # Step
        self.world_handling.step()

        # Get state

        # Get intent

        # Get spatial

        # Get simulation information

        # Prepare return




    