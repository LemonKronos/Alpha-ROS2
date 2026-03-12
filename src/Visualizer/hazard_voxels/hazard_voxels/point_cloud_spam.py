#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from python_utils.utils import *

class TripleCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_triple_camera_publisher')
        
        # Matching your exact drone topics
        self.pub_front = self.create_publisher(PointCloud2, Topic.DEPTH_CAM_FRONT_PL, 10)
        self.pub_left = self.create_publisher(PointCloud2, Topic.DEPTH_CAM_LEFT_PL, 10)
        self.pub_right = self.create_publisher(PointCloud2, Topic.DEPTH_CAM_RIGHT_PL, 10)
        
        # Your exact message specs
        self.height = 240
        self.width = 320
        self.point_step = 24
        self.row_step = 7680
        self.data_size = self.height * self.row_step # 1,843,200 bytes per camera
        
        self.msg = PointCloud2()
        self.msg.height = self.height
        self.msg.width = self.width
        self.msg.is_dense = False
        self.msg.is_bigendian = False
        self.msg.point_step = self.point_step
        self.msg.row_step = self.row_step
        
        self.msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
            PointField(name='rgb', offset=16, datatype=7, count=1),
        ]
        
        # 30 Hz timer (1.0 / 30.0 seconds)
        self.timer = self.create_timer(Clock.LOOP_CYCLE, self.timer_callback)
        self.get_logger().info('Blasting 3 depth cameras at 30Hz with RANDOMIZED points! CPU is gonna sweat.')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.msg.header.stamp = now
        
        # Generate fresh random bytes for the front camera
        self.msg.header.frame_id = "alpha_minus_2_0/base_link/alpha_depth_cam_front"
        self.msg.data = np.random.bytes(self.data_size)
        self.pub_front.publish(self.msg)
        
        # Generate fresh random bytes for the left camera
        self.msg.header.frame_id = "alpha_minus_2_0/base_link/alpha_depth_cam_left"
        self.msg.data = np.random.bytes(self.data_size)
        self.pub_left.publish(self.msg)
        
        # Generate fresh random bytes for the right camera
        self.msg.header.frame_id = "alpha_minus_2_0/base_link/alpha_depth_cam_right"
        self.msg.data = np.random.bytes(self.data_size)
        self.pub_right.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = TripleCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()