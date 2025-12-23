#!/usr/bin/env python3
import sys
import os

# --- NUCLEAR PATH FIX ---
# Add the PARENT directory of the brain library to sys.path
brain_src_path = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/src"
if os.path.exists(brain_src_path) and brain_src_path not in sys.path:
    sys.path.append(brain_src_path)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

# ROS Messages
from sensor_msgs.msg import Image
from ros2_msgs.msg import ControlInterface, FusePerception, Lidar2dObstacle

# Python Libs
import torch
import numpy as np
import threading
import time
import json

# Global Utils
import python_utils.utils as sys_config

# --- EXTERNAL BRAIN IMPORT ---
from acrobatic_brain.models.diffusion_noise_v2 import DiffusionPlanner

# --- LOCAL MODULES ---
from acrobatic_oa.normalizer import RoboticNormalizer
import acrobatic_oa.acrobatic_utils as brain_utils 

class BrainNode(Node):
    def __init__(self):
        super().__init__('acrobatic_oa')

        # --- 1. Simulation Clock Setup ---
        sys_config.setup_for_simulation(self)

        # --- 2. Hardcoded Paths (As requested) ---
        config_path = '/home/mr_lemon/MyCode/Project/Drone/AIBrain/src/acrobatic_brain/weights/planner/acrobatic_brain_config.json'
        ckpt_path = '/home/mr_lemon/MyCode/Project/Drone/AIBrain/src/acrobatic_brain/weights/planner/final_acrobatic_brain.ckpt'

        if not os.path.exists(config_path) or not os.path.exists(ckpt_path):
            self.get_logger().error(f"Path Error! Files not found.\nCfg: {config_path}\nCkpt: {ckpt_path}")
            return

        # --- Load Brain ---
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)

        self.get_logger().info(f"Loading Brain from: {ckpt_path}")
        self.model = DiffusionPlanner.load_from_checkpoint(
            ckpt_path, strict=False, **self.config['model_architecture']
        )
        self.model.to(self.device)
        self.model.eval()
        
        # --- Init Robotic Normalizer ---
        self.normalizer = RoboticNormalizer(self.device)
        self.get_logger().info(f"Brain Ready on {self.device}")

        # Inference Hyperparams
        self.seq_len = self.config['inference_hyperparams']['seq_len']
        self.smooth_kernel = self.config['inference_hyperparams']['smooth_kernel']
        
        # --- Buffers (Thread Safe) ---
        self.lock = threading.Lock()
        
        # Raw Data Caches
        self.cache_perc = None      # [16]
        self.cache_lidar_c = None   # [12]
        self.cache_lidar_f = None   # [12]
        self.cache_noob = None      # [7]
        self.cache_depth = None     # Tensor [1, 1, 480, 640]
        
        # Execution Plan
        self.current_plan = None    # [32, 7] Numpy (Normalized [-1, 1])
        self.plan_idx = 0
        self.safety_trigger = False # Flag for NaN panic

        # --- ROS Setup ---
        self.bridge = CvBridge()
        
        # QoS Settings
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subs
        self.create_subscription(ControlInterface, sys_config.CONTROL_INPUT_TOPIC, self.cb_noob, qos_reliable)
        self.create_subscription(FusePerception, sys_config.FUSE_PERCEPTION_TOPIC, self.cb_perception, qos_sensor)
        self.create_subscription(Lidar2dObstacle, sys_config.LIDAR_2D_CONTOUR_CLOSE_TOPIC, self.cb_lidar_close, qos_sensor)
        self.create_subscription(Lidar2dObstacle, sys_config.LIDAR_2D_CONTOUR_FAR_TOPIC, self.cb_lidar_far, qos_sensor)
        self.create_subscription(Image, sys_config.DEPTH_CAM_TOPIC, self.cb_depth, qos_sensor)

        # Pubs
        self.pub_control = self.create_publisher(ControlInterface, sys_config.CONTROL_ACROBATIC_TOPIC, qos_reliable)

        # --- Threads ---
        # 1. Inference Thread (GPU)
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

        # 2. Control Output Timer (80Hz)
        self.create_timer(sys_config.SYSTEM_CYCLE_FAST, self.stream_control)

    # --- Callbacks ---
    def cb_noob(self, msg):
        vec = [msg.roll, msg.pitch, msg.yaw, msg.forward, msg.left, msg.up, float(msg.wings_mode)]
        with self.lock:
            self.cache_noob = np.array(vec, dtype=np.float32)

    def cb_perception(self, msg):
        vec = []
        vec.extend(msg.position)
        vec.extend(msg.q)
        vec.extend(msg.velocity)
        vec.extend(msg.angular_velocity)
        vec.append(1.0 if msg.bearable_contact else 0.0)
        vec.append(1.0 if msg.critical_contact else 0.0)
        vec.append(msg.below_distance)
        with self.lock:
            self.cache_perc = np.array(vec, dtype=np.float32)

    def _extract_lidar(self, msg):
        sectors = np.full(sys_config.LIDAR_2D_SECTOR_NUM, sys_config.LIDAR_2D_RANGE_MAX, dtype=np.float32)
        for obs in msg.obstacles:
            if obs.sector_index < sys_config.LIDAR_2D_SECTOR_NUM:
                sectors[obs.sector_index] = obs.min_distance
        return sectors

    def cb_lidar_close(self, msg):
        vec = self._extract_lidar(msg)
        with self.lock:
            self.cache_lidar_c = vec

    def cb_lidar_far(self, msg):
        vec = self._extract_lidar(msg)
        with self.lock:
            self.cache_lidar_f = vec

    def cb_depth(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            cv_img = np.nan_to_num(cv_img, nan=sys_config.DEPTH_MAX_DIST)
            cv_img[cv_img > sys_config.DEPTH_MAX_DIST] = sys_config.DEPTH_MAX_DIST
            
            norm_img = cv_img / sys_config.DEPTH_MAX_DIST 
            tensor = torch.from_numpy(norm_img).float().unsqueeze(0).unsqueeze(0)
            
            with self.lock:
                self.cache_depth = tensor
        except Exception:
            pass

    # --- Inference Logic ---
    def build_raw_state(self):
        with self.lock:
            if self.cache_perc is None:
                self.get_logger().info("Waiting for Perception Data...", throttle_duration_sec=2.0)
                return None, None
            if self.cache_noob is None:
                self.get_logger().info("Waiting for Noob Control Input...", throttle_duration_sec=2.0)
                return None, None
            
            l_close = self.cache_lidar_c if self.cache_lidar_c is not None else np.full(12, 30.0)
            l_far = self.cache_lidar_f if self.cache_lidar_f is not None else np.full(12, 30.0)
            
            if brain_utils.has_nan(self.cache_perc) or brain_utils.has_nan(self.cache_noob) or \
               brain_utils.has_nan(l_close) or brain_utils.has_nan(l_far):
                self.safety_trigger = True
                self.get_logger().error("NaN DETECTED IN SENSORS! TRIGGERING HOVER.")
                return None, None
            else:
                self.safety_trigger = False

            state = np.concatenate([self.cache_perc, l_close, l_far])
            noob = self.cache_noob
            return state, noob

    def inference_loop(self):
        self.get_logger().info("Inference Loop Started. Waiting for data flow...")
        while rclpy.ok():
            state_np, noob_np = self.build_raw_state()
            
            if state_np is None:
                time.sleep(0.01)
                continue

            with self.lock:
                depth_t = self.cache_depth

            if depth_t is None:
                self.get_logger().info("Waiting for Depth Camera...", throttle_duration_sec=2.0)
                time.sleep(0.01)
                continue

            state_norm = self.normalizer.normalize_state(state_np)
            noob_norm = self.normalizer.normalize_noob(noob_np)
            depth_t = depth_t.to(self.device)

            try:
                start_t = time.time()
                traj_norm = self.model.sample(
                    state=state_norm,
                    noob=noob_norm,
                    img=depth_t,
                    seq_len=self.seq_len,
                    smooth_kernel=self.smooth_kernel
                )
                dt = time.time() - start_t
                
                traj_safe = self.normalizer.denormalize_action(traj_norm)
                plan_cpu = traj_safe.detach().cpu().numpy()[0]
                
                with self.lock:
                    self.current_plan = plan_cpu
                    self.plan_idx = 0 
                
                # Log successful inference stats
                self.get_logger().info(f"Inference Updated: {dt*1000:.1f}ms | Plan Length: {self.seq_len}", throttle_duration_sec=2.0)

            except Exception as e:
                self.get_logger().error(f"Brain Inference Failed: {e}")

    def stream_control(self):
        if self.safety_trigger:
            self.pub_control.publish(brain_utils.get_safety_hover_msg(self.get_clock().now()))
            self.get_logger().warn("SAFETY TRIGGER ACTIVE: Hovering...", throttle_duration_sec=1.0)
            return

        action = None
        with self.lock:
            if self.current_plan is None:
                self.get_logger().warn("Brain not ready (No Plan yet)...", throttle_duration_sec=2.0)
                return

            action = self.current_plan[self.plan_idx]
            
            if self.plan_idx < self.seq_len - 1:
                self.plan_idx += 1
            else:
                 self.get_logger().warn("Trajectory Exhausted! Holding last frame (Inference Lag?)", throttle_duration_sec=0.5)
        
        if action is not None:
            self.publish_action(action)

    def publish_action(self, action):
        msg = ControlInterface()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "acrobatic_brain"
        msg.control_by = ControlInterface.ACROBATIC_OA
        msg.control_state = True
        
        msg.roll = float(action[0])
        msg.pitch = float(action[1])
        msg.yaw = float(action[2])
        msg.forward = float(action[3])
        msg.left = float(action[4])
        msg.up = float(action[5])
        msg.wings_mode = max(0, min(2, int(round(action[6]))))
        
        self.pub_control.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()