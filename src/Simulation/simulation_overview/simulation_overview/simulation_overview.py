#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import dearpygui.dearpygui as dpg

# Custom imports
from alpha_msgs.msg import RecordControl
from python_utils.utils import *

# DPG Texture Settings
TEX_WIDTH = 1280
TEX_HEIGHT = 720

class SimulationOverviewNode(Node):
    def __init__(self, debug=False):
        super().__init__('simulation_overview')
        self.bridge = CvBridge()
        self.is_paused = False
        self.debug = debug

        # 1. Image Subscription (Best Effort)
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_img = self.create_subscription(
            Image,
            Topic.OVERVIEW_CAM,
            self.img_callback,
            qos_profile_img
        )

        # 2. Record Control Subscription (Reliable)
        qos_profile_record = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_record_control = self.create_subscription(
            RecordControl,
            Topic.LOGGER_RECORD,
            self.record_control_callback,
            qos_profile_record
        )

        self.get_logger().info("Simulation Overview Python Node Started (DPG Background Thread).")
        if self.debug:
            self.get_logger().info("Debug mode enabled.")

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (TEX_WIDTH, TEX_HEIGHT))

            if self.is_paused:
                cv2.putText(cv_image, "PAUSED", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            rgba_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGBA)
            texture_data = (rgba_image.astype(np.float32) / 255.0).flatten()

            if dpg.is_dearpygui_running() and dpg.does_alias_exist("fpv_texture"):
                dpg.set_value("fpv_texture", texture_data)

        except Exception as e:
            self.get_logger().error(f"Image processing exception: {e}")

    def record_control_callback(self, msg):
        self.is_paused = msg.pause

def spin_ros(node):
    """Runs the ROS2 event loop in a background thread."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Catch both just in case the signal reaches the thread directly
        pass

def main(args=None):
    rclpy.init(args=args)
    
    # Your clean arg parsing structure
    passed_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    is_debug = len(passed_args) > 1 and passed_args[1] == 'debug'
    
    node = SimulationOverviewNode(debug=is_debug)

    # Start ROS thread
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    # --- Setup DearPyGui (Main Thread) ---
    dpg.create_context()

    blank_data = np.zeros(TEX_WIDTH * TEX_HEIGHT * 4, dtype=np.float32)
    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(width=TEX_WIDTH, height=TEX_HEIGHT, default_value=blank_data, tag="fpv_texture")

    with dpg.window(label=WINDOW_OVERVIEW_FPV, tag="main_window", no_collapse=True):
        dpg.add_text(WINDOW_OVERVIEW_FPV, color=[0, 255, 150])
        dpg.add_image("fpv_texture")

    dpg.create_viewport(title=WINDOW_OVERVIEW_FPV, width=TEX_WIDTH + 50, height=TEX_HEIGHT + 100)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)

    # Your clean try/except/finally structure adapted for DPG
    try:
        dpg.start_dearpygui()
    except KeyboardInterrupt:
        pass
    finally:
        dpg.destroy_context()        # Close the GUI cleanly
        node.destroy_node()          # Clean up node FIRST
        if rclpy.ok():               # Check if ROS is already shutting down
            rclpy.shutdown()            
        ros_thread.join(timeout=1.0) # Ensure thread exits

if __name__ == '__main__':
    main()