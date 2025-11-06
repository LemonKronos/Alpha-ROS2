from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pygame_control',
            executable='pygame_control',
            name='pygame_control_node',
            output='screen'
        ),
        Node(
            package='finalize_control',
            executable='finalize_control',
            name='finalize_control_node',
            output='screen'
        ),
    ])
