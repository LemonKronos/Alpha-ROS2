import rclpy
import math
import numpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data

from python_utils.utils import *
from alpha_msgs.msg import VectorFieldHistogram
from alpha_msgs.msg import FusePerception


class VFHHandler:
    """Handles a single VFH stream entirely in its own isolated space."""
    def __init__(self, node: Node, sub_topic, pub_topic, name):
        self.node = node
        self.name = name

        # State variables (now safely isolated per stream!)
        self.has_vfh_counter = Threshold.ALLOW_MISSED_TOPIC
        self.has_vfh = False

        # Publisher
        self.publisher = self.node.create_publisher(Marker, pub_topic, 10)

        # Subscriber
        self.subscriber = self.node.create_subscription(
            VectorFieldHistogram, 
            sub_topic, 
            self.vfh_callback, 
            qos_profile_sensor_data
        )

        # Timer
        self.timer = self.node.create_timer(Clock.LOOP_CYCLE, self.timer_callback) # Sim clock


    def vfh_callback(self, msg: VectorFieldHistogram):
        self.has_vfh_counter = Threshold.ALLOW_MISSED_TOPIC
        self.has_vfh = True

        self.last_frame_id = msg.header.frame_id # Save this for the timer!

        marker = Marker()
        
        # Pass the header straight through for frame_id and timestamp
        marker.header = msg.header
        marker.ns = self.name
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Dynamic scale based on hazard distance
        scale_factor = 2 * math.tan(Sensor.VFH_RESOLUTION / 2) * self.node.hazard_distance
        marker.scale.x = scale_factor
        marker.scale.y = scale_factor
        marker.scale.z = scale_factor
        
        # Color
        marker.color.r, marker.color.g, marker.color.b = self.giveColor(self.name)
        marker.color.a = 0.6
        
        # Read VFH
        VFH_part = numpy.array(msg.vfh_part, dtype=numpy.int32)
        VFH = ((VFH_part[:, None] & (1 << numpy.arange(32))) > 0).flatten().astype(numpy.uint8)

        # Make list point
        for i, bin_val in enumerate(VFH):
            if bin_val == 1:
                cube = Point()

                row = i // Sensor.VFH_AZIMUTH_BINS
                col = i % Sensor.VFH_AZIMUTH_BINS

                yaw = col * Sensor.VFH_RESOLUTION - math.pi + Sensor.VFH_RESOLUTION/2
                pitch = row * Sensor.VFH_RESOLUTION - math.pi/2 + Sensor.VFH_RESOLUTION/2

                distance_cos_phi = self.node.hazard_distance * math.cos(pitch)

                cube.x = distance_cos_phi * math.cos(yaw)
                cube.y = distance_cos_phi * math.sin(yaw)
                cube.z = -self.node.hazard_distance * math.sin(pitch)

                marker.points.append(cube)
        
        # Publish
        self.publisher.publish(marker)

        percentage = len(marker.points) / Sensor.VFH_TOTAL_BINS * 100
        if percentage < 33:
            msg_color = GREEN
        elif percentage < 66:
            msg_color = YELLOW
        else:
            msg_color = RED
        
        self.node.get_logger().info(f"{msg_color} VFH {self.name} show {percentage:.2f}% occupied {RESET}")

    
    def giveColor(self, name):
        if name == "seeing": # YELLOW
            return (1.0, 1.0, 0.0)
        elif name == "memory": # TEAL
            return (0.0, 1.0, 1.0)
        elif name == "total": # SOFT RED
            return (0.8, 0.3, 0.3)


    def timer_callback(self):
        if self.has_vfh_counter > 0:
            self.has_vfh_counter -= 1
        elif self.has_vfh == True:
            self.has_vfh = False

            marker = Marker()
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.header.frame_id = getattr(self, 'last_frame_id', 'world') # Fallback to 'world' just in case
            marker.ns = self.name
            marker.id = 0
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.DELETEALL

            self.publisher.publish(marker)


class VFHSphereVisualizer(Node):
    def __init__(self):
        super().__init__('vfh_sphere_visualizer')

        Global.setup_for_simulation(self)

        self.hazard_distance = Drone.HAZARD_DISTANCE
        
        # 2. Setup the isolated streams
        self.seeing_stream = VFHHandler(
            node=self,
            sub_topic=Topic.VFH_HAZARD_SEEING,
            pub_topic='/visualizer/vfh_seeing_markers',
            name='seeing'
        )

        self.memory_stream = VFHHandler(
            node=self,
            sub_topic=Topic.VFH_HAZARD_MEMORY,
            pub_topic='/visualizer/vfh_memory_markers',
            name='memory'
        )

        self.total_stream = VFHHandler(
            node=self,
            sub_topic="/visualizer/total_vfh",
            pub_topic='visualize/vfh_total_markers',
            name='total'
        )

        # 3. Setup shared subscribers
        self.fuse_perception_SUB = self.create_subscription(
            FusePerception,
            Topic.FUSE_PERCEPTION,
            self.fuse_perception_callback,
            qos_profile_sensor_data
        )

    def fuse_perception_callback(self, msg: FusePerception):
        self.hazard_distance = msg.hazard_distance


def main(args=None):
    rclpy.init(args=args)
    node = VFHSphereVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()