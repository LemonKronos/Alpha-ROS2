#!/usr/bin/env python3
"""
record_acrobatic.py

ROS2 node to record and ouput dataset for AcrobaticOA

episode structure:
    episode_0001/
    ├─ state.npy
    ├─ action.npy
    ├─ noob_01.npy
    ├─ overview.mp4
    └─ meta.json

 - state.npy: sensor data, also include camera feed
 - action.npy: expert control
 - noob_xx.npy: noob control - also a state data, will later be concatenate to state.npy, can have multiple noob control for a expert action
 - overview.mp4: video record of the whole episode, use as a reference for adding nood control and a overview of the movement
 - meta.json: include meta data for the episode:
    - manuever: name for such expert action
    - fps: system rate
    - start: timestamp start episode
    - end: timestamp end episode

TODO: Update the code to do what we describe above
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import json
import time

class RecordAcrobaticNode(Node):
    def __init__(self):
        super().__init__('record_acrobatic_node')

        # Dataset config
        self.dataset_root = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/expert_data"
        os.makedirs(self.dataset_root, exist_ok=True)

        # Recording state
        self.recording = False
        self.episode_dir = None
        self.state_list = []
        self.action_list = []
        self.timestamps = []
        self.frame_idx = 0
        self.fps = 30
        self.bridge = CvBridge()
        self.video_writer = None
        self.width, self.height = 640, 480

        # ROS subscriptions
        self.create_subscription(Bool, '/record_control', self.cb_record_control, 10)
        self.create_subscription(Image, '/overview/camera/image_raw', self.cb_camera, 10)
        self.create_subscription(Image, '/drone/state', self.cb_state, 10)
        self.create_subscription(Image, '/drone/expert_action', self.cb_expert, 10)

    # ------------------ Control ------------------ #
    def cb_record_control(self, msg: Bool):
        if msg.data and not self.recording:
            self.start_episode()
        elif not msg.data and self.recording:
            self.finish_episode()

    def start_episode(self):
        timestamp = int(time.time())
        self.episode_dir = os.path.join(self.dataset_root, f"episode_{timestamp}")
        os.makedirs(self.episode_dir, exist_ok=True)
        self.state_list, self.action_list, self.timestamps = [], [], []
        self.frame_idx = 0
        video_path = os.path.join(self.episode_dir, "overview.mp4")
        self.video_writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (self.width, self.height))
        self.start_time = time.time()
        self.recording = True
        self.get_logger().info(f"Recording episode: {self.episode_dir}")

    def finish_episode(self):
        self.recording = False
        self.video_writer.release()

        np.save(os.path.join(self.episode_dir, "state.npy"), np.array(self.state_list, dtype=np.float32))
        np.save(os.path.join(self.episode_dir, "action.npy"), np.array(self.action_list, dtype=np.float32))
        np.save(os.path.join(self.episode_dir, "timestamp.npy"), np.array(self.timestamps, dtype=np.float64))

        meta = {
            "start_time": self.start_time,
            "end_time": time.time(),
            "fps": self.fps,
            "num_frames": len(self.timestamps)
        }
        with open(os.path.join(self.episode_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info("Episode saved.")

    # ------------------ Callbacks ------------------ #
    def cb_camera(self, msg: Image):
        if not self.recording:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.video_writer.write(frame)
        self.timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.frame_idx += 1

    def cb_state(self, msg):
        if self.recording:
            self.state_list.append(self.msg_to_list(msg))

    def cb_expert(self, msg):
        if self.recording:
            self.action_list.append(self.msg_to_list(msg))

    # ------------------ Utility ------------------ #
    def msg_to_list(self, msg):
        return [float(getattr(msg, f)) for f in msg.__slots__ if hasattr(msg, f)]

def main(args=None):
    rclpy.init(args=args)
    node = RecordAcrobaticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Silence Ctrl+C
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()

