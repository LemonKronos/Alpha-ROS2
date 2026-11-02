import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    low_spatial_params = os.path.join(
        get_package_share_directory('low_spatial'),
        'config',
        'low_spatial.yaml'
    )

    low_spatial_node = ComposableNode(
        package='low_spatial',
        plugin='spatial_container::LowSpatialNode',
        name='low_spatial',
        parameters=[low_spatial_params]
    )

    depth_cam_node = ComposableNode(
        package='depth_cam',
        plugin='spatial_container::DepthCamNode',
        name='depth_cam'
    )

    omni_lidar_node = ComposableNode(
        package='omni_lidar',
        plugin='spatial_container::OmniLidarNode',
        name='omni_lidar'
    )

    spatial_container = ComposableNodeContainer(
        name='spatial_container',
        namespace='spatial',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            #! Order matter here
            low_spatial_node,
            depth_cam_node,
            omni_lidar_node
        ],
        ros_arguments=[
            '--log-level', 'low_spatial:=warn',
            '--log-level', 'depth_cam:=warn',
            '--log-level', 'omni_lidar:=warn'
        ],
        output='screen',
        # prefix=['gdb -ex run --args'], #: DEBUG
    )

    return LaunchDescription([spatial_container])
