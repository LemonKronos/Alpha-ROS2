import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_path = os.path.join(os.path.dirname(__file__), 'static_tf_launch.py')
    spatial_observation_path = os.path.join(os.path.dirname(__file__), 'spatial_observation.py')

    training_params = os.path.join(
        get_package_share_directory('global_utils'),
        'config',
        'acrobatic_training.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spatial_observation_path)),
        Node(
            package='finalize_control',
            executable='finalize_control',
            name='finalize_control',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='fuse_perception',
            executable='fuse_perception',
            name='fuse_perception',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
         Node(
            package='contact_parser',
            executable='contact_parser',
            name='contact_parser',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='gz_ground_truth',
            executable='gz_ground_truth',
            name='gz_ground_truth',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='px4_emulator',
            executable='px4_emulator',
            name='px4_emulator',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='acrobatic_oa',
            executable='acrobatic_oa',
            name='acrobatic_oa',
            # arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[training_params]
        )
    ])
