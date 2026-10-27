import rclpy
from rclpy.node import Node

import numpy as np

from python_utils.utils import *
from alpha_msgs.msg import ControlInterface, FusePerception, NormalizedMotorsServos, SpatialObservation


class AcrobaticOANode(Node):
    def __init__(self):
        super().__init__('acrobatic_oa')

        #: Set up to use simulation clock
        Global.setup_for_simulation(self)
        
        #: Subscribers
        self.SUB_spatial = self.create_subscription(
            SpatialObservation,
            Topic.OBSERVATION_BOXES,
            self.spatial_callback,
            10
        )

        self.SUB_intent = self.create_subscription(
            ControlInterface,
            Topic.CONTROL_INPUT,
            self.intent_callback,
            10
        )

        self.SUB_state = self.create_subscription(
            FusePerception,
            Topic.FUSE_PERCEPTION,
            self.state_callback,
            10
        )

        #: Publishers
        self.PUB_acrobatic = self.create_publisher(NormalizedMotorsServos, Topic.CONTROL_ACROBATIC, 10)

        #: Data Buffers
        
        
        #: Timers
        self.TIME_inference = self.create_timer(0.01, self.inference_loop)


    def spatial_callback(self, msg):
        self.latest_spatial = msg


    def intent_callback(self, msg):
        self.latest_intent = msg


    def state_callback(self, msg):
        self.latest_state = msg


    def inference_loop(self):
        #: Lost input check
        #TODO 

        #: Inference
        # TODO Import and call AI core module 
        # Example: 
        # action = my_ai_core.infer(
        #     self.latest_spatial,  
        #     self.latest_state, 
        #     self.latest_intent
        # )

        dummy_action = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        
        msg = NormalizedMotorsServos()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.motors_control = dummy_action[:4]
        msg.servos_control = dummy_action[4:]

        self.PUB_acrobatic.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AcrobaticOANode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()