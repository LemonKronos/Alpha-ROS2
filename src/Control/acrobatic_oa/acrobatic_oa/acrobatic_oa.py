import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from python_utils.utils import *
from alpha_msgs.msg import ControlInterface, FusePerception


class AcrobaticOANode(Node):
    def __init__(self):
        super().__init__('acrobatic_oa')

        Global.setup_for_simulation(self)
        
        #: Subscribers
        #DEV We will use message filter - synchronizer for this
        #DEV coarse and detail box SUB

        self.intent_SUB = self.create_subscription(ControlInterface, Topic.CONTROL_INPUT, self.intent_cb, 10)

        self.state_SUB = self.create_subscription(FusePerception, Topic.FUSE_PERCEPTION, self.state_cb, 10)


        #: Publishers
        self.acrobatic_control_PUB = self.create_publisher(Float32MultiArray, Topic.CONTROL_ACROBATIC, 10)

        #: Data Buffers
        
        
        #: Timers
        self.timer = self.create_timer(0.01, self.inference_loop)


    def coarse_cb(self, msg):
        self.latest_coarse = msg

    def detail_cb(self, msg):
        self.latest_detail = msg

    def intent_cb(self, msg):
        self.latest_intent = msg

    def state_cb(self, msg):
        self.latest_state = msg

    def inference_loop(self):
        # Require state and intent to proceed
        if self.latest_state is None or self.latest_intent is None:
            return

        # TODO: Import and call your external AI workspace here
        # Example: 
        # action = my_ai_core.infer(
        #     self.latest_coarse, 
        #     self.latest_detail, 
        #     self.latest_state, 
        #     self.latest_intent
        # )

        # Dummy action for vertical slice pipeline testing (4 motors, 2 servos neutral)
        msg = Float32MultiArray()
        msg.data = [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]
        self.acrobatic_control_PUB.publish(msg)

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