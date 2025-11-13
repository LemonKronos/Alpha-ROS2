import rclpy
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ros2_msgs.msg import Lidar2dObstacle
from rclpy.qos import qos_profile_sensor_data


class RvizContour(Node):
    def __init__(self):
        super().__init__('rviz_contour')

        # Set up both pairs easily
        self._setup_contour_pair(
            sub_topic='/on_drone/sensor/scan/lidar2d/far',
            pub_topic='/visualizer/contour_marker_array/far',
            name='far'
        )

        self._setup_contour_pair(
            sub_topic='/on_drone/sensor/scan/lidar2d/close',
            pub_topic='/visualizer/contour_marker_array/close',
            name='close'
        )

    def _setup_contour_pair(self, sub_topic, pub_topic, name):
        """
        Sets up a subscriber+publisher pair with given topics and color.
        """
        publisher = self.create_publisher(MarkerArray, pub_topic, 10)
        callback = lambda msg, pub=publisher, ns=name: self._contour_callback(msg, pub, ns)
        self.create_subscription(Lidar2dObstacle, sub_topic, callback, qos_profile_sensor_data)

    def _contour_callback(self, msg, publisher, ns):
        marker_array = MarkerArray()
        sum_point = 0
        min_marker_points = 2000
        num_single = 0
        index = 0
        obstacles = msg.obstacles
        for sector in obstacles:
            point_array = []
            for contour in sector.contours:
                for point in contour.points:
                    point_array.append(Point(x=point.x, y=point.y, z=0.0))
                
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = index
            index += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.points = point_array


            if len(point_array) <= min_marker_points:
                if len(point_array) == 1:
                    num_single += 1
                min_marker_points = len(point_array)

            marker.color.r, marker.color.g, marker.color.b = self.giveColor(ns)
            marker.scale.x = 0.3
            marker.scale.y = 0.3

            marker.color.a = 1.0
            marker_array.markers.append(marker)

            sum_point += len(point_array)

        publisher.publish(marker_array)
        self.get_logger().info(f"Published {len(marker_array.markers)} {ns} obstacle contours, combine {sum_point} points.\n"
                               f"Number of single point = {num_single}, min contour size = {min_marker_points}")
        
        
    
    def giveColor(self, ns):
        if ns == "far": # Green
            # r = random.uniform(0.0, 0.4)
            # g = random.uniform(0.7, 0.7)
            # b = random.uniform(0.0, 0.4)
            return(0.0, 0.7, 0.0)
        else: # Red
            # r = random.uniform(0.8, 1.0)
            # g = random.uniform(0.0, 0.4)
            # b = random.uniform(0.0, 0.4)
            return(0.7, 0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = RvizContour()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Silence Ctrl+C
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
