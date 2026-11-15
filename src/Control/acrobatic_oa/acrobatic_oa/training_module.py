
import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.node import Node
from acrobatic_oa.environment import GzEnvironment

from alpha_ai_core.interface import EpisodeConfiguration, SpawnCondition, Action, StepResult, TaskType

import time
MAX_LINEAR_SPEED = 50.0   # m/s, total speed
MAX_ANGULAR_SPEED = 6.0  # rad/s, total rate
MAX_TILT_ANGLE = 1.57    # rad, angle between body z-axis and world z-axis

def random_vector(rng: np.random.Generator, max_norm: float) -> np.ndarray:
    """Random direction, magnitude uniform in [0, max_norm]. Total norm never exceeds max_norm."""
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    return direction * rng.uniform(0.0, max_norm)


def random_spawn_attitude(rng: np.random.Generator, max_tilt: float) -> list[float]:
    """
    Returns [roll, pitch, yaw] compatible with Rotation.from_euler('xyz', ...).
    Tilt (body z vs world z) is in [0, max_tilt], tilt direction and yaw are uniform over 360 deg.
    """
    tilt = rng.uniform(0.0, max_tilt)
    tilt_azimuth = rng.uniform(0.0, 2.0 * np.pi)
    yaw = rng.uniform(0.0, 2.0 * np.pi)

    tilt_axis = np.array([np.cos(tilt_azimuth), np.sin(tilt_azimuth), 0.0])
    rotation = Rotation.from_euler('z', yaw) * Rotation.from_rotvec(tilt * tilt_axis)
    return rotation.as_euler('xyz').tolist()


def random_spawn_condition(rng: np.random.Generator, position) -> SpawnCondition:
    return SpawnCondition(
        position=list(position),
        linear_velocity=random_vector(rng, MAX_LINEAR_SPEED).tolist(),
        angular_velocity=random_vector(rng, MAX_ANGULAR_SPEED).tolist(),
        roll_pitch_yaw=random_spawn_attitude(rng, MAX_TILT_ANGLE),
    )

class TrainingModule:
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.gz_environment = GzEnvironment(the_node)
        self.step_result: StepResult = None


    def training_loop(self):
        """
        Repeatedly run reset, then 20 step, try to populate the 
        """
        # : Test train loop
        time.sleep(1)
        rng = np.random.default_rng()
        while True:
            spawn = random_spawn_condition(rng, position=[0.0, 0.0, 3.0])
            ep_conf = EpisodeConfiguration(
                task=TaskType.HOVER,
                seed=9,
                spawn=spawn,
                condition_index=0,
                repeat_index=0,
                episode_index=0
            )
            #: Reset
            self.step_result = self.gz_environment.reset(episode_config=ep_conf)
            self.the_node.get_logger().info(f"{self.step_result}")

            #: Step
            counter = 0.0
            flip = -1
            for _ in range(0, 30):
                if flip > 0 and counter >= 1.0:
                    flip = -1
                elif flip < 0 and counter <= -1.0:
                    flip = 1

                counter += 0.1 * flip
                dummy_actions = np.array([counter, counter, counter, counter, counter, counter, -counter, -counter])

                self.step_result = self.gz_environment.step(Action(values=dummy_actions))
                self.the_node.get_logger().info(f"{self.step_result}")
                # time.sleep(1)

            # time.sleep(2)


        


            
    