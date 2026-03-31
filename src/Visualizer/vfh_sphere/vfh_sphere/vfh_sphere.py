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

class VFHSphereVisualizer(Node):
    def __init__(self):
        super().__init__('vfh_sphere_visualizer')

        Global.setup_for_simulation(self)

        # Setup both
        self.setup(
            sub_topic=Topic.VFH_HAZARD_SEEING,
            pub_topic='/visualizer/vfh_seeing_markers',
            name='seeing'
        )

        self.setup(
            sub_topic=Topic.VFH_HAZARD_MEMORY,
            pub_topic='/visualizer/vfh_memory_markers',
            name='memory'
        )

        # Subscribers
        self.fuse_perception_SUB = self.create_subscription(
            FusePerception,
            Topic.FUSE_PERCEPTION,
            self.fuse_perception_callback,
            qos_profile_sensor_data
        )

        # Init variables
        self.hazard_distance = Drone.HAZARD_DISTANCE;
    
    def setup(self, sub_topic, pub_topic, name):
        publisher = self.create_publisher(Marker, pub_topic, 10)
        callback = lambda msg, publisher=publisher, name = name: self.vfh_callback(msg, publisher, name)
        self.create_subscription(VectorFieldHistogram, sub_topic, callback, qos_profile_sensor_data)
        

    def vfh_callback(self, msg: VectorFieldHistogram, publisher, name):
        marker = Marker()
        
        # Pass the header straight through for frame_id and timestamp
        marker.header = msg.header
        marker.ns = name
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Dynamic scale base on hazard distance
        scale_factor = 2 * math.tan(Sensor.VFH_RESOLUTION / 2) * self.hazard_distance
        marker.scale.x = scale_factor
        marker.scale.y = scale_factor
        marker.scale.z = scale_factor
        
        # Color
        marker.color.r, marker.color.g, marker.color.b = self.giveColor(name)
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

                distance_cos_phi = self.hazard_distance * math.cos(pitch)

                cube.x = distance_cos_phi * math.cos(yaw)
                cube.y = distance_cos_phi * math.sin(yaw)
                cube.z = -self.hazard_distance * math.sin(pitch)

                marker.points.append(cube)
        
        # Publish
        publisher.publish(marker)

        percentage = len(marker.points) / Sensor.VFH_TOTAL_BINS * 100
        if percentage < 33:
            msg_color = GREEN
        elif percentage < 66:
            msg_color = YELLOW
        else:
            msg_color = RED
        self.get_logger().info(f"{msg_color} VFH show {percentage:.2f}% occupied {RESET}")

    
    def giveColor(self, name):
        if name == "seeing": # YELLOW
            return (1.0, 1.0, 0.0)
        elif name == "memory": # ORANGE
            return (1.0, 0.36, 0.0)


    def fuse_perception_callback(self, msg: FusePerception):
        self.hazard_distance =  msg.hazard_distance


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