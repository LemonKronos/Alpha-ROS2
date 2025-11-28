import rclpy 
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ################################################ Parameter
# System
SYSTEM_CYCLE = 1.0/30.0

# Topic path
CONTROL_INPUT_TOPIC = "/on_drone/drone_control/input/control"
CONTROL_CORRECTION_TOPIC = "/on_drone/drone_control/correction/control"  # inactive
CONTROL_FINAL_TOPIC = "/on_drone/drone_control/final/control"
FUSE_PERCEPTION_TOPIC = "/on_drone/sensor/fuse_perception"
CONTACT_PARSER_TOPIC = "/on_drone/sensor/body_contact"
LOGGER_RECORD_TOPIC = "/on_drone/logger/record_control"
LIDAR_2D_CONTOUR_CLOSE_TOPIC = "/on_drone/sensor/lidar2d/close/contour"
LIDAR_2D_CONTOUR_FAR_TOPIC = "/on_drone/sensor/lidar2d/far/contour"

# Topic path: visualizer
VISUAL_CONTROL_VEC_TOPIC = "/visualizer/control_vector"
VISUAL_MOVEMENT_VEC_TOPIC = "/visualizer/movement_vector"
VISUAL_REPULSIVE_VEC_TOPIC = "/visualizer/repulsive_vector"
VISUAL_CORRECTION_VEC_TOPIC = "/visualizer/correction_vector"

# Service path
CONTROL_WORLD_GRASSLAND = "/world/grasslands/control"


# ################################################ Function

def setup_for_simulation(target_node: Node, timeout_sec: float = 2.0):
    """
    Checks for the presence of the /clock topic by spinning a temporary node.
    If a message is received within the timeout, 'use_sim_time' is set to True.
    """
    # 1. Check for manual override (If user set use_sim_time=false, respect it)
    if target_node.has_parameter("use_sim_time"):
        if target_node.get_parameter("use_sim_time").value == False:
            return

    target_node.get_logger().info("Checking for Gazebo Clock (Timeout: %s sec)..." % timeout_sec)

    # 2. Create a temporary detector node
    detector = rclpy.create_node('clock_detector_temp')
    
    # QoS must match /clock (Best effort is required for clock reliability)
    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # 3. Use a flag container (Python list trick for mutable boolean)
    clock_found = [False]

    def clk_cb(msg):
        # Callback runs when a message is received
        clock_found[0] = True
        # Note: We don't need to destroy the subscription here, 
        # as it will be destroyed when the 'detector' node is destroyed.

    # 4. Create the subscription (The "Ear")
    sub = detector.create_subscription(Clock, '/clock', clk_cb, qos_profile)

    # 5. Spin the detector for a bit (Blocking)
    # This processes the discovery traffic and checks for the message.
    rclpy.spin_once(detector, timeout_sec=timeout_sec)

    # 6. Logic and Parameter Setting
    if clock_found[0]:
        target_node.get_logger().warn("Clock detected! Mode: SIMULATION")
        if target_node.has_parameter("use_sim_time"):
            # Set existing parameter
            target_node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        else:
            # Declare new parameter
            target_node.declare_parameter("use_sim_time", True)
    else:
        target_node.get_logger().info("No Clock detected. Mode: REALTIME")
        # Ensure it's declared false if it wasn't there before
        if not target_node.has_parameter("use_sim_time"):
             target_node.declare_parameter("use_sim_time", False)
    
    # 7. Cleanup
    detector.destroy_node()
    