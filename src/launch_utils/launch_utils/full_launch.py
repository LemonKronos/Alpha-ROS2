import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_path = os.path.join(os.path.dirname(__file__), 'static_tf_launch.py')
    spatial_container_path = os.path.join(os.path.dirname(__file__), 'spatial_container_launch.py')

    foam_visualizer_params = os.path.join(
        get_package_share_directory('global_utils'),
        'config',
        'alpha-foam.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_tf_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spatial_container_path)),
        Node(
            package='html_controller',
            executable='html_controller',
            name='html_controller',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='simulation_control',
            executable='simulation_control',
            name='simulation_control',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        # Node(
        #     package='simulation_overview',
        #     executable='simulation_overview',
        #     name='simulation_overview',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', 'warn']
        # ),
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
            package='foam_visualizer',
            executable='foam_visualizer',
            name='foam_visualizer',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[foam_visualizer_params]
        )
    ])
