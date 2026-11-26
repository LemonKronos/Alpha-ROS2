#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node

from python_utils.utils import *

from gz.transport13 import Node as GZNode
from gz.msgs10.wind_pb2 import Wind


WIND_TOPIC = "/world/training_ground/wind"

WIND_SEQUENCE = [
    (5.0, 0.0, 0.0),
    (0.0, 5.0, 0.0),
    (-5.0, 0.0, 0.0),
    (0.0, -5.0, 0.0),
]


def count_open_descriptors():
    """Return (total descriptors, socket descriptors) of this process."""
    total_count = 0
    socket_count = 0
    for descriptor_name in os.listdir("/proc/self/fd"):
        try:
            target = os.readlink(f"/proc/self/fd/{descriptor_name}")
        except OSError:
            continue
        total_count += 1
        if target.startswith("socket:"):
            socket_count += 1
    return total_count, socket_count


class WindTestNode(Node):
    def __init__(self):
        super().__init__("wind_test_node")

        #: Set up to use simulation clock
        # Global.setup_for_simulation(self)

        self.declare_parameter("change_period_seconds", 5.0)
        change_period_seconds = (
            self.get_parameter("change_period_seconds").get_parameter_value().double_value
        )

        self.gz_node = GZNode()
        self.wind_publisher = self.gz_node.advertise(WIND_TOPIC, Wind)
        if not self.wind_publisher:
            raise RuntimeError(f"Failed to advertise {WIND_TOPIC}")

        self.sequence_index = 0
        self.start_descriptor_count, self.start_socket_count = count_open_descriptors()
        self.get_logger().info(
            f"Start: descriptors={self.start_descriptor_count}, "
            f"sockets={self.start_socket_count}"
        )

        self.timer = self.create_timer(change_period_seconds, self.on_timer)

    def set_wind(self, wind_x: float, wind_y: float, wind_z: float = 0.0):
        wind = Wind()
        wind.enable_wind = True
        wind.linear_velocity.x = wind_x
        wind.linear_velocity.y = wind_y
        wind.linear_velocity.z = wind_z

        if not self.wind_publisher.publish(wind):
            self.get_logger().warning("Failed to publish wind!")
        else:
            self.get_logger().info(
                f"Wind set to: X={wind_x}, Y={wind_y}, Z={wind_z} m/s"
            )

    def on_timer(self):
        wind_x, wind_y, wind_z = WIND_SEQUENCE[self.sequence_index]
        self.sequence_index = (self.sequence_index + 1) % len(WIND_SEQUENCE)
        self.set_wind(wind_x, wind_y, wind_z)

        descriptor_count, socket_count = count_open_descriptors()
        self.get_logger().info(
            f"Descriptors: total={descriptor_count} "
            f"(+{descriptor_count - self.start_descriptor_count}), "
            f"sockets={socket_count} "
            f"(+{socket_count - self.start_socket_count})"
        )


def main():
    rclpy.init()
    wind_test_node = WindTestNode()
    try:
        rclpy.spin(wind_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        wind_test_node.set_wind(0.0, 0.0, 0.0)
        wind_test_node.get_logger().info("Wind disabled.")
        wind_test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()