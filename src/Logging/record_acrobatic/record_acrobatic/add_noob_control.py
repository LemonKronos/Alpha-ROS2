# add_noob_control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import cv2
import os
import json
import time

class AddNoobControlNode(Node):
    def __init__(self):
        super().__init__('add_noob_control_node')

        # Episode to annotate
        self.episode_dir = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/expert_data/episode_0001"
        self.overview_path = os.path.join(self.episode_dir, "overview.mp4")
        self.noob_file = os.path.join(self.episode_dir, "noob_01.npy")

        # Load metadata / timestamps
        self.timestamps = np.load(os.path.join(self.episode_dir, "timestamp.npy"))
        self.state_array = np.load(os.path.join(self.episode_dir, "state.npy"))
        self.action_array = np.load(os.path.join(self.episode_dir, "action.npy"))

        # Storage for noob input
        self.noob_inputs = []

        # ROS publishers (optional)
        self.noob_pub = self.create_publisher(Float32MultiArray, '/drone/noob_action', 10)

        # Start playback
        self.playback_episode()

    # ------------------ Playback & record ------------------ #
    def playback_episode(self):
        cap = cv2.VideoCapture(self.overview_path)
        frame_idx = 0
        total_frames = len(self.timestamps)
        self.get_logger().info(f"Playing back {total_frames} frames")

        while cap.isOpened() and frame_idx < total_frames:
            ret, frame = cap.read()
            if not ret:
                break

            # TODO: capture your noob input here, e.g., via pygame keys
            noob_action = self.get_noob_input(frame_idx)

            self.noob_inputs.append(noob_action)

            # Optional: publish live
            msg = Float32MultiArray(data=noob_action)
            self.noob_pub.publish(msg)

            frame_idx += 1
            cv2.imshow("Overview Playback", frame)
            if cv2.waitKey(int(1000/30)) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        np.save(self.noob_file, np.array(self.noob_inputs, dtype=np.float32))
        self.get_logger().info(f"Noob input saved to {self.noob_file}")

    # ------------------ Stub for noob input ------------------ #
    def get_noob_input(self, idx):
        # Replace with pygame or joystick input
        # Example: zero action
        return [0.0, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = AddNoobControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Silence Ctrl+C
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
