from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pygame_control',
            executable='pygame_control',
            name='pygame_control_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='finalize_control',
            executable='finalize_control',
            name='finalize_control_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='reactive_oa',
            executable='reactive_oa',
            name='reactive_oa_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'warn']
        ),
                Node(
            package='fuse_perception',
            executable='fuse_perception',
            name='fuse_perception_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
