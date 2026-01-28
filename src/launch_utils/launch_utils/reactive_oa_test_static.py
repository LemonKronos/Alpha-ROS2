from launch import LaunchDescription
from launch_ros.actions import Node
import launch.logging

# launch.logging.get_logger().setLevel('WARN')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='advance_control',
            executable='advance_control',
            name='advance_control_node',
            output='screen',
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