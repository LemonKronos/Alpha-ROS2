#!/usr/bin/env python3
# --- Example ROS2 Node Integration ---
"""
advance_control.py
ROS2 node for advance 6 DoF input from a phone through web socket
"""
import rclpy
import argparse
import socket
import sys
from rclpy.node import Node
from rclpy.clock import Clock
import rclpy.clock 

from advance_control.web_socket_controller import WebSocketController
from alpha_msgs.msg import RecordControl
from alpha_msgs.msg import ControlInterface
from python_utils.utils import *

# Get machine IP as default
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
DEFAULT_IP = s.getsockname()[0]
s.close()

DEFAULT_PORT = 8765

class AdvanceControlNode(Node):
    def __init__(self, debug = False, ip = DEFAULT_IP, port = DEFAULT_PORT):
        super().__init__('advance_control_node')

        Global.setup_for_simulation(self)
        self.do_debug = debug

        # 1. Setup Wall Clock (Thread-safe, steady)
        self.wall_clock = rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
    
        # 2. Start Web Controller
        self.is_connected = False
        self.last_is_connected = False
        self.web_controller = WebSocketController(host=ip, port=port)
        self.web_controller.start()
        
        # 3. Publishers
        self.control_interface_PUB = self.create_publisher(ControlInterface, Topic.CONTROL_INPUT, 10)
        self.record_control_PUB = self.create_publisher(RecordControl, Topic.LOGGER_RECORD, 10)
        
        # 4. State Tracking
        self.last_record = False
        self.last_pause = False
        self.no_input = True 
        self.no_input_log = True

        # 5. Timers
        self.timer_control = self.create_timer(Clock.LOOP_CYCLE_FAST, self.timer_control_callback)
        self.timer_realtime = self.create_timer(Clock.LOOP_CYCLE, self.timer_realtime_callback, clock=self.wall_clock)

        self.get_logger().info(f"Control server on {f"{YELLOW}Default{RESET} " if ip is DEFAULT_IP else ''}IP: {GREEN}{ip}{RESET}, {f"{YELLOW}Default{RESET} " if port is DEFAULT_PORT else ''}port: {GREEN}{port}{RESET}")
        self.get_logger().warn(f"{YELLOW} ⏳ Wating for controller... {RESET}")

    def timer_control_callback(self):
        commands = self.web_controller.get_latest_control_cmd()

        control_msg = ControlInterface()
        control_msg.control_by = ControlInterface.OPERATOR
        control_msg.control_state = bool(commands.get('control_state', False))
        control_msg.forward = float(commands.get('x', 0.0))
        control_msg.left    = float(commands.get('y', 0.0))
        control_msg.up      = float(commands.get('z', 0.0))
        control_msg.roll    = float(commands.get('roll', 0.0))
        control_msg.pitch   = float(commands.get('pitch', 0.0))
        control_msg.yaw     = float(commands.get('yaw', 0.0))
        control_msg.wings_mode = ControlInterface.UNCHANGE
        control_msg.header.stamp = self.get_clock().now().to_msg()

        self.control_interface_PUB.publish(control_msg)

        if self.do_debug:
            if self.no_input_log and self.no_input:
                self.get_logger().info(f"{'🎮' if control_msg.control_state else '❌'}Forwar _.__🔼   Left _.__◀️   Up _.__⬆️   Roll _.__🔄   Pitch _.__↕️   Yaw _.__↔️")
                self.no_input_log = False

            if control_msg.forward != 0 or control_msg.left != 0 or control_msg.up != 0 or control_msg.roll != 0 or control_msg.pitch != 0 or control_msg.yaw != 0:
                self.get_logger().info(f"{'🎮' if control_msg.control_state else '❌'}Forwar{control_msg.forward:> 5.2f}🔼   Left{control_msg.left:> 5.2f}◀️   Up{control_msg.up:> 5.2f}⬆️   Roll{control_msg.roll:> 5.2f}🔄   Pitch{control_msg.pitch:> 5.2f}↕️   Yaw{control_msg.yaw:> 5.2f}↔️")
                self.no_input = False
                self.no_input_log = True

            else:
                self.no_input = True

    def timer_realtime_callback(self):
        self.is_connected = self.web_controller.check_is_connected()
        if self.is_connected != self.last_is_connected:
            self.last_is_connected = self.is_connected
            if self.is_connected:
                self.get_logger().warn(f"{GREEN} 🔌 Controller connected {RESET}")
            else:
                self.get_logger().warn(f"{RED} ❌ Controller disconnected {RESET}")

        commands = self.web_controller.get_latest_record_cmd()

        new_record = commands.get('record', False)
        new_pause  = commands.get('pause', False)

        if new_record != self.last_record or new_pause != self.last_pause:
            flag = RecordControl()
            flag.record = bool(new_record)
            flag.pause  = bool(new_pause)

            self.record_control_PUB.publish(flag)

            self.last_record = new_record
            self.last_pause  = new_pause

            if False:
                if new_record:
                    self.get_logger().info( f"{GREEN}Start record{RESET}")
                else:
                    self.get_logger().info(f"{GREEN}Stop record{RESET}")

                if new_pause:
                    self.get_logger().info(f"{YELLOW}Pause record...{RESET}")
                else:
                    self.get_logger().info(f"{YELLOW}Continue record...{RESET}")

    def destroy_node(self):
        self.web_controller.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    passed_args = rclpy.utilities.remove_ros_args(args=sys.argv)

    parser = argparse.ArgumentParser(description='Advance Control Node')
    parser.add_argument('--debug', action='store_true', help='Enable debug print')
    parser.add_argument('--ip', type=str, default=DEFAULT_IP, help='Target IPv4')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='Target port number')
    parser, _ = parser.parse_known_args(passed_args[1:])

    node = AdvanceControlNode(debug=parser.debug, ip=parser.ip, port=parser.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()