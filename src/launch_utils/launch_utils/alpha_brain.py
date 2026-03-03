import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='alpha_brain',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='octo_map',
                plugin='alpha_brain::OctoMapPlugin',
                name='octo_map_component'
            ),
            ComposableNode(
                package='high_spacial',
                plugin='alpha_brain::HighSpacialPlugin',
                name='high_spacial_component'
            ),
            ComposableNode(
                package='reactive_oa',
                plugin='alpha_brain::ReactiveOAPlugin',
                name='reactive_oa_component'
            ),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])