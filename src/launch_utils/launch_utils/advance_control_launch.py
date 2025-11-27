from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='advance_control',
            executable='advance_control',
            name='advance_control_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='finalize_control',
            executable='finalize_control',
            name='finalize_control_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
         Node(
            package='contact_parser',
            executable='contact_parser',
            name='contact_parser_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='fuse_perception',
            executable='fuse_perception',
            name='fuse_perception_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
