
from rclpy.node import Node
from acrobatic_oa.environment import GzEnvironment

class TrainingModule:
    def __init__(self, the_node: Node):
        self.the_node = the_node
        self.gz_environment = GzEnvironment(the_node)

    def training_loop(self):
        #: Test
        from alpha_ai_core.interface import EpisodeConfiguration, TaskType, SpawnCondition
        import time
        ep_conf = EpisodeConfiguration(
            task=TaskType.HOVER,
            seed=0,
            reinitialize_vehicle=True,
            spawn=SpawnCondition(
                position=[0.0, 0.0, 3.0],
                linear_velocity=[5.0, 5.0, 5.0],
                angular_velocity=[0.0, 0.0, 0.0],
                roll_pitch_yaw=[0.785, 0.15, 0.3]
            ),
            condition_index=0,
            repeat_index=0,
            episode_index=0
        )
        time.sleep(1)
        while True:
            self.gz_environment.reset(episode_config=ep_conf)
            time.sleep(5)
    