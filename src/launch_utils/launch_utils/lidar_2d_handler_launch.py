from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_2d_handler',
            executable='lidar_2d_handler',
            name='lidar_2d_handler_node',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'x500_lidar_2d_0/link/lidar_2d_v2'],
        ),
    ])
