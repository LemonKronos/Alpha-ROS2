import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from python_utils.utils import *

from alpha_msgs.msg import VoxelBlock

class HazardVoxelVisualizer(Node):
    def __init__(self):
        super().__init__('hazard_voxel_visualizer')

        Global.setup_for_simulation(self)
        
        # Publisher for RViz
        self.marker_pub = self.create_publisher(
            Marker,
            '/visualizer/hazard_voxel_markers',
            10
        )
        
        self.voxel_sub = self.create_subscription(
            VoxelBlock,
            '/visualizer/hazard_voxel',
            self.voxel_callback,
            qos_profile_sensor_data
            
        )
        
        self.get_logger().info("Hazard Voxel Visualizer Node (Python) is up and chilling.")

    def voxel_callback(self, msg):
        self.get_logger().info("Get hazard points")

        marker = Marker()
        
        # Pass the header straight through for frame_id and timestamp
        marker.header = msg.header
        marker.ns = "hazard_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Hardcoded resolution/scale - adjust this to match your octomap resolution!
        marker.scale.x = Sensor.VOXEL_RESOLUTION
        marker.scale.y = Sensor.VOXEL_RESOLUTION
        marker.scale.z = Sensor.VOXEL_RESOLUTION
        
        # Color: Semi-transparent Orange for hazards
        marker.color.r = 1.0
        marker.color.g = 0.36
        marker.color.b = 0.0
        marker.color.a = 0.6
        
        # Convert Point32 to float64 Point for RViz Marker
        # Iterating through the point_array from your custom message
        for pt32 in msg.point_array.points:
            pt = Point()
            pt.x = float(pt32.x)
            pt.y = float(pt32.y)
            pt.z = float(pt32.z)
            marker.points.append(pt)
            
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = HazardVoxelVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()