import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from ros2_msgs.msg import Lidar2dObstacle
from rclpy.qos import qos_profile_sensor_data


class RvizContourPoint(Node):
    def __init__(self):
        super().__init__('rviz_contour')

        self._setup_contour_pair(
            sub_topic='/on_drone/sensor/scan/lidar2d/far',
            pub_topic='/visualizer/contour_marker/far',
            color=(0.0, 1.0, 0.0),
            name='far'
        )

        self._setup_contour_pair(
            sub_topic='/on_drone/sensor/scan/lidar2d/close',
            pub_topic='/visualizer/contour_marker/close',
            color=(1.0, 0.0, 0.0),
            name='close'
        )

    def _setup_contour_pair(self, sub_topic, pub_topic, color, name):
        publisher = self.create_publisher(Marker, pub_topic, 10)
        callback = lambda msg, pub=publisher, col=color, ns=name: self._contour_callback(msg, pub, col, ns)
        self.create_subscription(Lidar2dObstacle, sub_topic, callback, qos_profile_sensor_data)

    def _contour_callback(self, msg, publisher, col, ns):
        point_array = []

        obstacles = msg.obstacles
        for sector in obstacles:
            for contour in sector.contours:
                for point in contour.points:
                    point_array.append(Point(x=point.x, y=point.y, z=0.0))

        if len(point_array) == 0:
            return

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = point_array

        if len(point_array) == 1:
            marker.scale.x = 0.9
            marker.scale.y = 0.9  
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.get_logger().info(f"[{ns}] Only 1 point detected.")
        else:
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.color.r, marker.color.g, marker.color.b = col
            marker.color.a = 1.0

        publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RvizContourPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Silence Ctrl+C
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
