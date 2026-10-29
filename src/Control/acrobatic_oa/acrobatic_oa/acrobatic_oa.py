import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.clock
from rclpy.clock import Clock

import numpy as np

from python_utils.utils import *
from alpha_msgs.msg import ControlInterface, FusePerception, NormalizedMotorsServos, SpatialObservation

from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose
)

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
            10
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
        self.PUB_acrobatic = self.create_publisher(NormalizedMotorsServos, Topic.CONTROL_ACROBATIC, 10)

        #: Data
        self.world_handling = WorldHandling(self)
        
        #: Timers
        # self.TIME_inference = self.create_timer(Clock.LOOP_CYCLE_FAST, self.inference_loop)

        #: Test _______________________________________________________

        # self.world_handling.pause()

        # self.world_handling.spawn_drone(
        #     10.0, 0.0, 1.0,
        #     1.0, 0.0, 0.0, 0.0
        # )

        # time.sleep(1.0)

        # self.world_handling.reset_time_in_pause(123)

        self.world_handling.teleport_drone_and_reset_time(
            10.0, -10.0, 1.0,
            1.0, 0.0, 0.0, 0.0,
            123
        )

        self.get_logger().info("Acrobatic node init", throttle_duration_sec=1)


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

        dummy_action = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0])
        
        msg = NormalizedMotorsServos()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.motors_control = dummy_action[:4]
        msg.servos_control = dummy_action[4:]

        self.PUB_acrobatic.publish(msg)

        self.get_logger().info("Published action",throttle_duration_sec=1)

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