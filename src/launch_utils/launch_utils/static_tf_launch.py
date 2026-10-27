from launch import LaunchDescription
from launch_ros.actions import Node
from python_utils.utils import Name


DRONE_TF_CONFIGS = {
    'alpha_minus_1': [
        # {
        #     'name': '',
        #     'xyz': ['', '', ''],
        #     'quat': ['', '', '', ''],
        #     'suffix': '',
        #     'child_suffix': ''
        # },
        {
            'name': 'depth_cam_front_static_tf',
            'xyz': ['0.5', '0', '0'],
            'quat': ['0.0', '0.0', '0.0', '1.0'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_front'
        },
        {
            'name': 'depth_cam_left_static_tf',
            'xyz': ['-0.158', '1.0', '0.0'],
            'quat': ['0.0', '0.0', '0.6087626432369819', '0.7933524085796444'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_left'
        },
        {
            'name': 'depth_cam_right_static_tf',
            'xyz': ['-0.158', '-1.0', '0.0'],
            'quat': ['0.0', '0.0', '-0.6087626432369819', '0.7933524085796444'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_right'
        },
        {
            'name': 'lidar_down_static_tf',
            'xyz': ['0.2', '0', '0.02'],
            'quat': ['0', '0.7071', '0', '0.7071'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_lidar_down'
        },
        {
            'name': 'alpha_lidar_2d_static_tf',
            'xyz': ['0', '0', '0.055'],
            'quat': ['0', '0', '0', '1'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_lidar_2d'
        },
    ],

    'alpha_minus_2': [
        {
            'name': 'depth_cam_front_static_tf',
            'xyz': ['0.5', '0', '0'],
            'quat': ['0.0', '0.0', '0.0', '1.0'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_front'
        },
        {
            'name': 'depth_cam_left_static_tf',
            'xyz': ['-0.158', '1.0', '0.0'],
            'quat': ['0.0', '0.0', '0.6087626432369819', '0.7933524085796444'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_left'
        },
        {
            'name': 'depth_cam_right_static_tf',
            'xyz': ['-0.158', '-1.0', '0.0'],
            'quat': ['0.0', '0.0', '-0.6087626432369819', '0.7933524085796444'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_depth_cam_right'
        },
        {
            'name': 'lidar_down_static_tf',
            'xyz': ['0.2', '0', '0.02'],
            'quat': ['0', '0.7071', '0', '0.7071'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_lidar_down'
        },
        {
            'name': 'alpha_lidar_2d_static_tf',
            'xyz': ['0', '0', '0.055'],
            'quat': ['0', '0', '0', '1'],
            'suffix': '/base_link',
            'child_suffix': '/alpha_lidar_2d'
        },
    ]
}

def generate_launch_description():
    drone_name = Name.Dynamic.DRONE().get()
    drone_base_link = Name.Dynamic.BASE_LINK().get()

    tf_data_list = DRONE_TF_CONFIGS.get(drone_name, [])
    tf_nodes = []

    for tf_data in tf_data_list:
        
        # Dynamically build the frame strings so they always match the swarm
        parent_frame = drone_base_link
        child_frame = f"{parent_frame}{tf_data['child_suffix']}"

        node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=tf_data['name'],
            ros_arguments=['--log-level', 'error'],
            arguments=[
                '--x', tf_data['xyz'][0], '--y', tf_data['xyz'][1], '--z', tf_data['xyz'][2],
                '--qx', tf_data['quat'][0], '--qy', tf_data['quat'][1], '--qz', tf_data['quat'][2], '--qw', tf_data['quat'][3],
                '--frame-id', parent_frame,
                '--child-frame-id', child_frame
            ]
        )
        tf_nodes.append(node)

    return LaunchDescription(tf_nodes)
    
