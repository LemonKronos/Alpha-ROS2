#!/usr/bin/env python3
"""
record_acrobatic.py
Records expert episodes for LDP Drone Project.
Features:
- Synchronized State (40 floats) and Action (7 floats) recording at 30Hz.
- Video recording for RGB, Depth, and Overview.
- Variable Lidar handling via 12-Sector downsampling.
"""

import rclpy
from rclpy.node import Node
from ros2_msgs.msg import ControlInterface, RecordControl, FusePerception, Lidar2dObstacle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import json
import time

# --- Configuration ---
SYSTEM_RATE = 30
SYSTEM_CYCLE = 1.0 / SYSTEM_RATE
TIMEOUT_CYCLES = 3       # Sensors dead if no msg for ~0.1s
LIDAR_SECTORS = 12       # Fixed size for AI input
LIDAR_MAX_DIST = 10.0    # Default value for empty sectors

class RecordAcrobaticNode(Node):
    def __init__(self):
        super().__init__('record_acrobatic_node')

        # Dataset Path
        self.dataset_root = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/simulated_expert"
        os.makedirs(self.dataset_root, exist_ok=True)

        # State Variables
        self.recording = False
        self.episode_dir = None
        self.state_list = []
        self.action_list = []
        self.start_timestamp = 0
        
        # Video Writers
        self.vw_overview = None
        self.vw_rgb = None
        self.vw_depth = None
        
        self.bridge = CvBridge()
        self.dim_input = (640, 480)     # RGB/Depth Resolution
        self.dim_overview = (1280, 720) # Overview Resolution

        # Data Cache (Stores latest available data)
        self.cache = {
            'perception': {'msg': None, 'time': 0},
            'lidar_close': {'msg': None, 'time': 0},
            'lidar_far':   {'msg': None, 'time': 0},
            'img_overview': None,
            'img_rgb': None,
            'img_depth': None
        }

        # Action Buffer (Default neutral)
        # [roll, pitch, yaw, fwd, left, up, wings_mode]
        self.current_action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # --- Subscriptions ---
        # Control & System
        self.create_subscription(RecordControl, "logger/record_control", self.record_control_callback, 10)
        self.create_subscription(ControlInterface, "control/input", self.expert_action_callback, 10)
        
        # Sensors
        self.create_subscription(FusePerception, "/on_drone/sensor/fuse_perception", self.perception_callback, 10)
        self.create_subscription(Lidar2dObstacle, "/on_drone/sensor/scan/lidar2d/close", self.lidar_close_callback, 10)
        self.create_subscription(Lidar2dObstacle, "/on_drone/sensor/scan/lidar2d/far", self.lidar_far_callback, 10)
        
        # Cameras
        self.create_subscription(Image, "sensor/depth_cam/camera/image", self.depth_cam_callback, 10)
        self.create_subscription(Image, "sensor/rgb_cam/camera/image", self.rgb_cam_callback, 10)
        self.create_subscription(Image, "sensor/overview_cam/camera/image", self.overview_cam_callback, 10)

        # Heartbeat Timer (30Hz)
        self.timer = self.create_timer(SYSTEM_CYCLE, self.node_loop_callback)
        self.get_logger().info("Recorder Ready. Trigger via 'logger/record_control'.")

    # ------------------ Node Loop (30Hz Heartbeat) -----------------------#
    def node_loop_callback(self):
        if not self.recording:
            return

        now = time.time()
        current_state_vector = []

        # --- 1. Perception (16 floats) ---
        p_data = self.cache['perception']
        if self.is_alive(p_data, now):
            msg = p_data['msg']
            # Pos(3) + Q(4) + Vel(3) + AngVel(3) + Cont(2) + Below(1)
            vec = list(msg.position) + list(msg.q) + list(msg.velocity) + list(msg.angular_velocity)
            vec.append(1.0 if msg.bearable_contact else 0.0)
            vec.append(1.0 if msg.critical_contact else 0.0)
            vec.append(msg.below_distance)
            current_state_vector.extend(vec)
        else:
            # Dead sensor filler
            current_state_vector.extend([0.0] * 16)

        # --- 2. Lidar Close (12 floats) ---
        l_close = self.cache['lidar_close']
        if self.is_alive(l_close, now):
            current_state_vector.extend(self.extract_sectors(l_close['msg']))
        else:
            current_state_vector.extend([LIDAR_MAX_DIST] * LIDAR_SECTORS)

        # --- 3. Lidar Far (12 floats) ---
        l_far = self.cache['lidar_far']
        if self.is_alive(l_far, now):
            current_state_vector.extend(self.extract_sectors(l_far['msg']))
        else:
            current_state_vector.extend([LIDAR_MAX_DIST] * LIDAR_SECTORS)

        # --- 4. Append Synchronized Data ---
        # Both lists grow at exactly 30Hz
        self.state_list.append(current_state_vector)
        self.action_list.append(self.current_action)

        # --- 5. Write Video Frames ---
        # Note: We write whatever is in the cache (Last Known Good Frame)
        # This keeps the video frame count mostly synced with the steps
        if self.vw_rgb and self.cache['img_rgb'] is not None:
            self.vw_rgb.write(self.cache['img_rgb'])
            
        if self.vw_depth and self.cache['img_depth'] is not None:
            self.vw_depth.write(self.cache['img_depth'])
            
        if self.vw_overview and self.cache['img_overview'] is not None:
            self.vw_overview.write(self.cache['img_overview'])

    # ------------------ Helpers ------------------ #
    def is_alive(self, data_dict, now):
        """Check if sensor data is fresh"""
        if data_dict['msg'] is None: return False
        if (now - data_dict['time']) > (SYSTEM_CYCLE * TIMEOUT_CYCLES): return False
        return True

    def extract_sectors(self, msg):
        """Convert Custom Lidar Msg to Fixed 12-float vector"""
        sectors = [LIDAR_MAX_DIST] * LIDAR_SECTORS
        for sec in msg.obstacles:
            idx = sec.sector_index
            if 0 <= idx < LIDAR_SECTORS:
                sectors[idx] = sec.min_distance
        return sectors

    def msg_to_action_list(self, msg):
        return [
            msg.roll, msg.pitch, msg.yaw,
            msg.forward, msg.left, msg.up,
            float(msg.wings_mode)
        ]

    # ------------------ Callbacks ------------------ #
    def perception_callback(self, msg):
        self.cache['perception']['msg'] = msg
        self.cache['perception']['time'] = time.time()

    def lidar_close_callback(self, msg):
        self.cache['lidar_close']['msg'] = msg
        self.cache['lidar_close']['time'] = time.time()
        
    def lidar_far_callback(self, msg):
        self.cache['lidar_far']['msg'] = msg
        self.cache['lidar_far']['time'] = time.time()

    def expert_action_callback(self, msg):
        # Update the buffer immediately, applied in next loop tick
        self.current_action = self.msg_to_action_list(msg)

    # --- Camera Callbacks (Just update cache) ---
    def rgb_cam_callback(self, msg):
        if not self.recording: return
        try: self.cache['img_rgb'] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def depth_cam_callback(self, msg):
        if not self.recording: return
        try:
            # Save depth as visualize-able grayscale BGR for MP4
            d_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            d_norm = cv2.normalize(d_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.cache['img_depth'] = cv2.cvtColor(d_norm, cv2.COLOR_GRAY2BGR)
        except: pass

    def overview_cam_callback(self, msg):
        if not self.recording: return
        try: self.cache['img_overview'] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    # ------------------ Record Management ------------------ #
    def record_control_callback(self, msg):
        if msg.record and not self.recording:
            self.start_episode()
        elif not msg.record and self.recording:
            self.finish_episode()

    def start_episode(self):
        ts = int(time.time())
        self.episode_dir = os.path.join(self.dataset_root, f"episode_{ts}")
        os.makedirs(self.episode_dir, exist_ok=True)
        
        self.state_list, self.action_list = [], []
        
        # Setup Video Writers
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.vw_rgb = cv2.VideoWriter(os.path.join(self.episode_dir, "rgb_front.mp4"), fourcc, SYSTEM_RATE, self.dim_input)
        self.vw_depth = cv2.VideoWriter(os.path.join(self.episode_dir, "depth_front.mp4"), fourcc, SYSTEM_RATE, self.dim_input, isColor=True)
        self.vw_overview = cv2.VideoWriter(os.path.join(self.episode_dir, "overview.mp4"), fourcc, SYSTEM_RATE, self.dim_overview)
        
        self.start_timestamp = time.time()
        self.recording = True
        self.get_logger().info(f"REC: Started episode_{ts}")

    def finish_episode(self):
        self.recording = False
        end_ts = time.time()
        
        # Cleanup
        if self.vw_rgb: self.vw_rgb.release()
        if self.vw_depth: self.vw_depth.release()
        if self.vw_overview: self.vw_overview.release()
        
        # Convert to Numpy
        state_arr = np.array(self.state_list, dtype=np.float32)
        action_arr = np.array(self.action_list, dtype=np.float32)
        
        # Verify Sync
        if state_arr.shape[0] != action_arr.shape[0]:
            self.get_logger().warn(f"SYNC WARNING: State {state_arr.shape} != Action {action_arr.shape}")
        
        np.save(os.path.join(self.episode_dir, "state.npy"), state_arr)
        np.save(os.path.join(self.episode_dir, "action.npy"), action_arr)
        
        # Metadata
        meta = {
            "expert_manuever": "unknown",
            "fps": SYSTEM_RATE,
            "duration": end_ts - self.start_timestamp,
            "frames": len(self.state_list),
            "state_dim": state_arr.shape[1],
            "action_dim": action_arr.shape[1]
        }
        with open(os.path.join(self.episode_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
            
        self.get_logger().info(f"REC: Saved episode_{int(self.start_timestamp)}. Frames: {len(self.state_list)}")

def main(args=None):
    rclpy.init(args=args)
    node = RecordAcrobaticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()