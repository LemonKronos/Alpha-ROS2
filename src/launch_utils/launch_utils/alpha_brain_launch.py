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
        plugin='alpha_brain::LowSpatialNode',
        name='low_spatial',
        parameters=[low_spatial_params],
        # arguments=['--ros-args', '--log-level', 'warn']
    )

    depth_cam_node = ComposableNode(
        package='depth_cam',
        plugin='alpha_brain::DepthCamNode',
        name='depth_cam',
        # arguments=['--ros-args', '--log-level', 'warn']
    )

    alpha_brain_container = ComposableNodeContainer(
        name='alpha_brain',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            low_spatial_node,
            depth_cam_node
        ],
        output='screen'
    )

    return LaunchDescription([alpha_brain_container])
