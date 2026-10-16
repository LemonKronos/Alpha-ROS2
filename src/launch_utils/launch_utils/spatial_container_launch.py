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

    #! Order matter here

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

    reactive_oa_node = ComposableNode(
        package='reactive_oa',
        plugin='spatial_container::ReactiveOANode',
        name='reactive_oa'
    )

    #DEV Acrobatic brain go here

    map_visualizer_node = ComposableNode(
        package='map_visualizer',
        plugin='spatial_container::MapVisualizerNode',
        name='map_visualizer'
    )

    spatial_container_container = ComposableNodeContainer(
        name='spatial_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            low_spatial_node,
            depth_cam_node,
            omni_lidar_node,
            reactive_oa_node,
            # map_visualizer_node
        ],
        # ros_arguments=[
        #     '--log-level',
        #     'low_spatial_node:=warn',
        #     'depth_cam_node:=warn',
        #     'omni_lidar_node:=warn'
        # ],
        output='screen',
        # prefix=['gdb -ex run --args'], #DEBUG
    )

    return LaunchDescription([spatial_container_container])
