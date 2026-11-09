import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.clock
from rclpy.clock import Clock

import numpy as np

from python_utils.utils import *
from alpha_msgs.msg import ControlInterface, FusePerception, NormalizedMotorsServos, SpatialObservation
from alpha_ai_core.interface import PolicyCore, SpatialEncoder
from alpha_ai_core.models import spatial_cnn, monolithic


# Hack to allow VS Code 'Run' button to see the module before colcon build
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from acrobatic_oa.world_handling import WorldHandling

class AcrobaticOANode(Node):
    def __init__(self):
        super().__init__('acrobatic_oa')

        #: Set up to use simulation clock
        Global.setup_for_simulation(self)
        
        #: Subscribers
        self.SUB_spatial = self.create_subscription(
            SpatialObservation,
            Topic.SPATIAL_OBSERVATION,
            self.spatial_callback,
            qos_profile_sensor_data
        )

        self.SUB_intent = self.create_subscription(
            ControlInterface,
            Topic.CONTROL_INPUT,
            self.intent_callback,
            qos_profile_sensor_data
        )

        self.SUB_state = self.create_subscription(
            FusePerception,
            Topic.FUSE_PERCEPTION,
            self.state_callback,
            qos_profile_sensor_data
        )

        #: Publishers
        self.PUB_acrobatic = self.create_publisher(NormalizedMotorsServos, Topic.CONTROL_ACROBATIC, qos_profile_sensor_data)

        #: Data
        self.policy_core: PolicyCore= monolithic()
        self.cnn_core = spatial_cnn()
        
        #: Timers
        self.TIME_inference = self.create_timer(1.0, self.inference_loop)


    def spatial_callback(self, msg):
        self.latest_spatial = msg


    def intent_callback(self, msg):
        self.latest_intent = msg


    def state_callback(self, msg):
        self.latest_state = msg


    def inference_loop(self):
        """This is the real/evaluation run"""
        # glob input the self.latest_spatial to np
        # self.cnn_core.encode(the spatial)

        # convert the itent and state
        # action, _, _ = self.policy_core.act(...) Normalize the action in the AI core code

        # action in here 8-DIM, all range [-1, 1], specific motor servo clamp get hangled in later node.
        dummy_action = np.array([0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0])
        
        msg = NormalizedMotorsServos()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.motors_control = dummy_action[:4]
        msg.servos_control = dummy_action[4:]

        self.PUB_acrobatic.publish(msg)

        self.get_logger().info(f"Published action {dummy_action}",throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = AcrobaticOANode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()