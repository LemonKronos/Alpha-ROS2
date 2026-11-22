# Hack to allow VS Code 'Run' button to see the module before colcon build
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import rclpy.clock
from rclpy.clock import Clock

from python_utils.utils import *
from alpha_msgs.msg import ControlInterface, FusePerception, NormalizedMotorsServos, SpatialObservation
from acrobatic_oa.toy_trainer import ToyTrainer
from acrobatic_oa.real_trainer import RealTrainer
from acrobatic_oa.evaluator import Evaluator
from alpha_ai_core.interface import FloatArray


class AcrobaticOANode(Node):
    def __init__(self):
        super().__init__('acrobatic_oa')

        #: Set up to use simulation clock
        Global.setup_for_simulation(self)

        #: Load ROS2 params
        self.declare_parameter('acrobatic_training', False)
        self.training = self.get_parameter('acrobatic_training').get_parameter_value().bool_value

        #: Publishers
        self.PUB_acrobatic = self.create_publisher(NormalizedMotorsServos, Topic.CONTROL_ACROBATIC, qos_profile_sensor_data)

        #: Data
        self.latest_intent = None
        self.latest_state = None
        self.latest_spatial = None
        self.trainer = None
        self.training_failed = False
        
        if self.training: #_Branch for training 
            self.get_logger().warn(f"{RED} Acrobatic training mode, remember to turn of PX4 {RESET}")
            self.trainer = None

            self.declare_parameter('training_mode', "real_training")
            self.training_mode = self.get_parameter('training_mode').get_parameter_value().string_value
            match self.training_mode:
                case "test_environment":
                    self.trainer = ToyTrainer(self)
                    self.get_logger().warn(f"{BLUE} ToyTrainer, testing environment... {RESET}")
                case "real_training":
                    self.trainer = RealTrainer(self)
                    self.get_logger().warn(f"{GREEN} RealTrainer, run training... {RESET}")
                case "evaluate":
                    self.trainer = Evaluator(self)
                    self.get_logger().warn(f"{YELLOW} Evaluator, grading model... {RESET}")

            threading.Thread(target=self.trainer.training_loop, daemon=True).start()
        else: #_Branch for running 
            self.get_logger().info(f"{BLUE} Acrobatic inference mode {RESET}")

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
            
            #: Data
            # self.policy_core: PolicyCore= monolithic()
            # self.cnn_core = spatial_cnn()

            #: Timers
            self.TIME_inference = self.create_timer(1.0, self.inference_loop)


    def spatial_callback(self, msg):
        self.latest_spatial = msg


    def intent_callback(self, msg):
        self.latest_intent = msg


    def state_callback(self, msg):
        self.latest_state = msg

    def request_shutdown_after_failure(self):
        """Called from the trainer thread when training cannot continue."""
        self.training_failed = True
        rclpy.try_shutdown()

    def publish_action(self, action: FloatArray):
        msg = NormalizedMotorsServos()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.motors_control = action[:4].astype(np.float32)
        msg.servos_control = action[4:].astype(np.float32)

        self.PUB_acrobatic.publish(msg)

        self.get_logger().info(f"Published action {action}",throttle_duration_sec=1)

    def inference_loop(self):
        """This is the real/evaluation run"""
        # glob input the self.latest_spatial to np
        # self.cnn_core.encode(the spatial)

        # convert the itent and state
        # action, _, _ = self.policy_core.act(...) Normalize the action in the AI core code

        # action in here 8-DIM, all range [-1, 1], specific motor servo clamp get hangled in later node.
        dummy_action = np.array([0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0])
        self.publish_action(dummy_action)
        

def main(args=None):
    rclpy.init(args=args)
    node = AcrobaticOANode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.trainer is not None:
            node.trainer.stop()
    finally:
        if node.training and getattr(node.trainer, "analyzer", None) is not None:
            node.trainer.analyzer.print_summary()
            
        node.destroy_node()
        rclpy.try_shutdown()

    if node.training_failed:
        sys.exit(1)

if __name__ == '__main__':
    main()