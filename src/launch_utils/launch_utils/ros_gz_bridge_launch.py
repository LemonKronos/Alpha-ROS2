from launch import LaunchDescription
from launch_ros.actions import Node
from python_utils.utils import Name

def generate_launch_description():
    # 1. Grab the dynamic variables
    drone_id = Name.Dynamic.DRONE_ID().get()
    world_name = Name.Dynamic.WORLD().get()

    # 2. Define the topics to bridge (Topics need type mapping and remapping)
    topics_to_bridge = [
        # Clock
        {"gz_topic": "/clock", "gz_type": "gz.msgs.Clock", "ros_topic": "/clock", "ros_type": "rosgraph_msgs/msg/Clock"},
        
        #_ dynamic transforms 
        # TF_static check
        # {"gz_topic": f"/model/{drone_id}/pose", "gz_type": "gz.msgs.Pose_V", "ros_topic": "/tf", "ros_type": "tf2_msgs/msg/TFMessage"},
        # {"gz_topic": f"/model/{drone_id}/pose_static", "gz_type": "gz.msgs.Pose_V", "ros_topic": "/tf_static", "ros_type": "tf2_msgs/msg/TFMessage"},

        # Lidar 1d & 2d as point clouds
        {"gz_topic": "/alpha_lidar_down/scan/points", "gz_type": "gz.msgs.PointCloudPacked", "ros_topic": "/sensor/lidar_1d_down/scan/points", "ros_type": "sensor_msgs/msg/PointCloud2"},
        {"gz_topic": "/alpha_lidar_2d/scan/points", "gz_type": "gz.msgs.PointCloudPacked", "ros_topic": "/sensor/lidar_2d/scan/points", "ros_type": "sensor_msgs/msg/PointCloud2"},
        
        # Depth camera x3
        {"gz_topic": "/alpha_depth_cam/front/camera/image/points", "gz_type": "gz.msgs.PointCloudPacked", "ros_topic": "/sensor/depth_cam/front/points", "ros_type": "sensor_msgs/msg/PointCloud2"},
        {"gz_topic": "/alpha_depth_cam/left/camera/image/points", "gz_type": "gz.msgs.PointCloudPacked", "ros_topic": "/sensor/depth_cam/left/points", "ros_type": "sensor_msgs/msg/PointCloud2"},
        {"gz_topic": "/alpha_depth_cam/right/camera/image/points", "gz_type": "gz.msgs.PointCloudPacked", "ros_topic": "/sensor/depth_cam/right/points", "ros_type": "sensor_msgs/msg/PointCloud2"},
        
        #_ extra cameras 
        # {"gz_topic": "/alpha_cam_front/camera/image", "gz_type": "gz.msgs.Image", "ros_topic": "/sensor/rgb_cam/camera/image", "ros_type": "sensor_msgs/msg/Image"},
        # {"gz_topic": "/gimbal/camera/image", "gz_type": "gz.msgs.Image", "ros_topic": "sensor/gimbal_cam/camera/image", "ros_type": "sensor_msgs/msg/Image"},
        # {"gz_topic": "/alpha_cam_overview/camera/image", "gz_type": "gz.msgs.Image", "ros_topic": "/sensor/overview_cam/camera/image", "ros_type": "sensor_msgs/msg/Image"},
        
        #_ dynamic contact sensors 
        {"gz_topic": f"/world/{world_name}/model/{drone_id}/link/base_link/sensor/alpha_body_contact/contact", "gz_type": "gz.msgs.Contacts", "ros_topic": "/sensor/contact_body/contact", "ros_type": "ros_gz_interfaces/msg/Contacts"},
        {"gz_topic": f"/world/{world_name}/model/{drone_id}/link/rotor_0/sensor/alpha_rotor_0_contact/contact", "gz_type": "gz.msgs.Contacts", "ros_topic": "/sensor/contact_rotor0/contact", "ros_type": "ros_gz_interfaces/msg/Contacts"},
        {"gz_topic": f"/world/{world_name}/model/{drone_id}/link/rotor_1/sensor/alpha_rotor_1_contact/contact", "gz_type": "gz.msgs.Contacts", "ros_topic": "/sensor/contact_rotor1/contact", "ros_type": "ros_gz_interfaces/msg/Contacts"},
        {"gz_topic": f"/world/{world_name}/model/{drone_id}/link/rotor_2/sensor/alpha_rotor_2_contact/contact", "gz_type": "gz.msgs.Contacts", "ros_topic": "/sensor/contact_rotor2/contact", "ros_type": "ros_gz_interfaces/msg/Contacts"},
        {"gz_topic": f"/world/{world_name}/model/{drone_id}/link/rotor_3/sensor/alpha_rotor_3_contact/contact", "gz_type": "gz.msgs.Contacts", "ros_topic": "/sensor/contact_rotor3/contact", "ros_type": "ros_gz_interfaces/msg/Contacts"},

        #_ world handling 
        {"gz_topic": f"/world/{world_name}/stats", "gz_type": "gz.msgs.WorldStatistics", "ros_topic": "/world/stats", "ros_type": "ros_gz_interfaces/msg/WorldStatistics"},
        {"gz_topic": f"/world/{world_name}/wrench", "gz_type": "gz.msgs.EntityWrench", "ros_topic": "/world/wrench", "ros_type": "ros_gz_interfaces/msg/EntityWrench", "direction": "]"},

        #_ px4 emulator 
        {"gz_topic": f"/{drone_id}/command/motor_speed", "gz_type": "gz.msgs.Actuators", "ros_topic": "/drone/motors", "ros_type": "actuator_msgs/msg/Actuators", "direction": "]"},
        {"gz_topic": f"/model/{drone_id}/servo_0", "gz_type": "gz.msgs.Double", "ros_topic": "/drone/elevon_left", "ros_type": "std_msgs/msg/Float64", "direction": "]"},
        {"gz_topic": f"/model/{drone_id}/servo_1", "gz_type": "gz.msgs.Double", "ros_topic": "/drone/elevon_right", "ros_type": "std_msgs/msg/Float64", "direction": "]"},
        {"gz_topic": f"/model/{drone_id}/servo_2", "gz_type": "gz.msgs.Double", "ros_topic": "/drone/tilt_right", "ros_type": "std_msgs/msg/Float64", "direction": "]"},
        {"gz_topic": f"/model/{drone_id}/servo_3", "gz_type": "gz.msgs.Double", "ros_topic": "/drone/tilt_left", "ros_type": "std_msgs/msg/Float64", "direction": "]"},

        #_ gz ground truth 
        {"gz_topic": f"/model/{drone_id}/odometry", "gz_type": "gz.msgs.Odometry", "ros_topic": "/drone/odometry", "ros_type": "nav_msgs/msg/Odometry"},

    ]

    # 3. Define the Services using a dictionary for auto-remapping
    services_to_bridge = [
        {"base_name": "control", "type": "ros_gz_interfaces/srv/ControlWorld"},
        {"base_name": "create", "type": "ros_gz_interfaces/srv/SpawnEntity"},
        {"base_name": "remove", "type": "ros_gz_interfaces/srv/DeleteEntity"},
        {"base_name": "set_pose", "type": "ros_gz_interfaces/srv/SetEntityPose"},
    ]

    bridge_args = []
    bridge_remappings = []

    # 4. Parse our topics into the bridge argument strings
    for topic in topics_to_bridge:
        # Format: gz_topic@ros_type<direction>gz_type
        # Default to Gazebo->ROS '[' if no direction is specified
        direction = topic.get('direction', '[')
        arg_string = f"{topic['gz_topic']}@{topic['ros_type']}{direction}{topic['gz_type']}"
        bridge_args.append(arg_string)
        
        # Remap if Gazebo and ROS topic names differ
        if topic['gz_topic'] != topic['ros_topic']:
            bridge_remappings.append((topic['gz_topic'], topic['ros_topic']))

    # 5. Parse our services and auto-generate the remappings
    for srv in services_to_bridge:
        # Build the dynamic Gazebo name (e.g., /world/grasslands/control)
        gz_srv = f"/world/{world_name}/{srv['base_name']}"
        
        # Build the clean ROS name (e.g., /world/control)
        ros_srv = f"/world/{srv['base_name']}"

        # Bridge argument format for services is simpler: gz_service@ros_type
        bridge_args.append(f"{gz_srv}@{srv['type']}")

        # Tell the node to remap the messy Gazebo name to your clean ROS name
        bridge_remappings.append((gz_srv, ros_srv))

    # 6. Create the Node, passing everything directly in memory
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=bridge_remappings
    )

    return LaunchDescription([bridge_node])