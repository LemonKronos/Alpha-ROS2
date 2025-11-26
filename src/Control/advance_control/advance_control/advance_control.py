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

# Constants
SYSTEM_CYCLE = 1/30

class AdvanceControlNode(Node):
    def __init__(self):
        super().__init__('advance_control_node')
        
        # Initialize and Start the WebSocket Controller
        self.web_controller = WebSocketController()
        self.web_controller.start()
        
        # Publisher
        self.control_interface_PUB = self.create_publisher(ControlInterface, "control/final", 10)
        self.record_control_PUB = self.create_publisher(RecordControl, "logger/record_control", 10)
        
        # Subscriber
        # For when we publish data to phone

        # Timer
        self.timer = self.create_timer(SYSTEM_CYCLE, self.timer_callback)

        # Init variable
        self.last_record = False
        self.last_pause = False

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
        if True:
            self.get_logger().info(f"🎮Forward:{control_msg.forward:> 6.2f} Left: {control_msg.left:> 6.2f} Up: {control_msg.up:> 6.2f} Roll: {control_msg.roll:> 6.2f} Pitch: {control_msg.pitch:> 6.2f} Yaw: {control_msg.yaw:> 6.2f}🛩️")

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
                self.get_logger().info("Start record")
            else:
                self.get_logger().info("Stop record")

            if new_pause:
                self.get_logger().info("Pause record...")
            else:
                self.get_logger().info("Continue record...")

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
