from launch import LaunchDescription
from launch_ros.actions import Node
import launch.logging

# launch.logging.get_logger().setLevel('WARN')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pygame_control',
            executable='pygame_control',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='lidar_2d_handler',
            executable='lidar_2d_handler',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='reactive_oa',
            executable='reactive_oa',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'x500_lidar_2d_0/link/lidar_2d_v2',
                       '--ros-args', '--log-level', 'warn'],
        ),
        Node(
            package='rviz_contour',
            executable='rviz_contour',
            name='rviz_contour',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='rviz_contour',
            executable='rviz_contour_points',
            name='rviz_contour_points',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])