from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='',
        #     arguments=[
        #         '--x', '', '--y', '', '--z', '',
        #         '--qx', '', '--qy', '', '--qz', '', '--qw', '',
        #         '--frame-id', '',
        #         '--child-frame-id', ''
        #         ]
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            ros_arguments=['--log-level', 'error'],
            name='depth_cam_front_static_tf',
            arguments=[
                '--x', '0.5', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'alpha_minus_2_0/base_link',
                '--child-frame-id', 'alpha_minus_2_0/base_link/alpha_depth_cam_front'
                ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            ros_arguments=['--log-level', 'error'],
            name='depth_cam_left_static_tf',
            arguments=[
                '--x', '-0.158', '--y', '1.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.6087626432369819', '--qw', '0.7933524085796444',
                '--frame-id', 'alpha_minus_2_0/base_link',
                '--child-frame-id', 'alpha_minus_2_0/base_link/alpha_depth_cam_left'
                ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            ros_arguments=['--log-level', 'error'],
            name='depth_cam_right_static_tf',
            arguments=[
                '--x', '-0.158', '--y', '-1.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '-0.6087626432369819', '--qw', '0.7933524085796444',
                '--frame-id', 'alpha_minus_2_0/base_link',
                '--child-frame-id', 'alpha_minus_2_0/base_link/alpha_depth_cam_right'
                ]
        ),
        
    ])