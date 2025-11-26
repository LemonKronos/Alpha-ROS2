from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    episode_arg = DeclareLaunchArgument(
        'episode',
        default_value='0',
        description='Episode number to inject noob control'
    )

    return LaunchDescription([
        episode_arg,

        Node(
            package='advance_control',
            executable='advance_control',
            name='advance_control_node',
            output='screen',
        ),
        Node(
            package='record_acrobatic',
            executable='add_noob_control',
            name='add_noob_control_node',
            output='screen',
            arguments=[LaunchConfiguration('episode')]
        )
    ])
