import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf_path = os.path.join(os.path.dirname(__file__), 'static_tf.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_path)
        ),
        Node(
            package='advance_control',
            executable='advance_control',
            name='advance_control',
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
        Node(
            package='finalize_control',
            executable='finalize_control',
            name='finalize_control',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='reactive_oa',
            executable='reactive_oa',
            name='reactive_oa',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='fuse_perception',
            executable='fuse_perception',
            name='fuse_perception',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='depth_cam',
            executable='depth_cam',
            name='depth_cam',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='lidar_2d_handler',
            executable='lidar_2d_handler',
            name='lidar_2d_handler',
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
        Node(
            package='hazard_voxels',
            executable='hazard_voxels',
            name='hazard_voxels',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
