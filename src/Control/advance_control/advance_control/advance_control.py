#!/usr/bin/env python3
# --- Example ROS2 Node Integration ---
"""

advance_control.py

ROS2 node for advance 6 DoF input from a phone through web socket
ros2_msgs::msg::ControlInterface messages to topic "control/final"
"""
import rclpy
from rclpy.node import Node
from advance_control.web_socket_controller import WebSocketController # Web control
from ros2_msgs.msg import RecordControl # Record flag
from ros2_msgs.msg import ControlInterface # Control interface
import python_utils.utils as utils

class AdvanceControlNode(Node):
    def __init__(self):
        super().__init__('advance_control_node')
        
        utils.setup_for_simulation(self)

        # Initialize and Start the WebSocket Controller
        self.web_controller = WebSocketController()
        self.web_controller.start()
        
        # Publisher
        self.control_interface_PUB = self.create_publisher(ControlInterface, utils.CONTROL_INPUT_TOPIC, 10)
        self.record_control_PUB = self.create_publisher(RecordControl, utils.LOGGER_RECORD_TOPIC, 10)
        
        # Subscriber
        # For when we publish data to phone

        # Timer
        self.timer = self.create_timer(utils.SYSTEM_CYCLE, self.timer_callback)

        # Init variable
        self.last_record = False
        self.last_pause = False
        self.no_input = True # For Logging
        self.no_input_log = True # For Logging

    def timer_callback(self):
        # Call web control to get lastest control input
        commands = self.web_controller.get_latest_commands()

        # Control msg, publish continuous stream
        control_msg = ControlInterface()

        control_msg.control_by = ControlInterface.HUMAN
        control_msg.control_state = True
        control_msg.forward = float(commands.get('x', 0.0))
        control_msg.left = float(commands.get('y', 0.0))
        control_msg.up = float(commands.get('z', 0.0))
        control_msg.roll = float(commands.get('roll', 0.0))
        control_msg.pitch = float(commands.get('pitch', 0.0))
        control_msg.yaw = float(commands.get('yaw', 0.0))
        control_msg.wings_mode = ControlInterface.UNCHANGE
        control_msg.header.stamp = self.get_clock().now().to_msg()

        self.control_interface_PUB.publish(control_msg)
        if False:
            if self.no_input_log and self.no_input:
                self.get_logger().info(" Forwar _.__🔼   Left _.__◀️   Up _.__⬆️   Roll _.__🔄   Pitch _.__↕️   Yaw _.__↔️")
                self.no_input_log = False

            if control_msg.forward != 0 or control_msg.left != 0 or control_msg.up != 0 or control_msg.roll != 0 or control_msg.pitch != 0 or control_msg.yaw != 0:
                self.get_logger().info(f" Forwar{control_msg.forward:> 5.2f}🔼   Left{control_msg.left:> 5.2f}◀️   Up{control_msg.up:> 5.2f}⬆️   Roll{control_msg.roll:> 5.2f}🔄   Pitch{control_msg.pitch:> 5.2f}↕️   Yaw{control_msg.yaw:> 5.2f}↔️")
                self.no_input = False
                self.no_input_log = True

            else:
                self.no_input = True

        # Record msg, publish when change
        new_record = commands.get('record', False)
        new_pause  = commands.get('pause', False)

        if new_record != self.last_record or new_pause != self.last_pause:
            flag = RecordControl()

            flag.record = bool(new_record)
            flag.pause  = bool(new_pause)

            self.record_control_PUB.publish(flag)

            self.last_record = new_record
            self.last_pause  = new_pause

            if new_record:
                self.get_logger().info( f"{GREEN}Start record{RESET}")
            else:
                self.get_logger().info(f"{GREEN}Stop record{RESET}")

            if new_pause:
                self.get_logger().info(f"{YELLOW}Pause record...{RESET}")
            else:
                self.get_logger().info(f"{YELLOW}Continue record...{RESET}")

    def destroy_node(self):
        # Stop the background thread when the node shuts down
        self.web_controller.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = AdvanceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
