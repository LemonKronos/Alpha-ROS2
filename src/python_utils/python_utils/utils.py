import time
from rclpy.parameter import Parameter

RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
PINK    = "\033[35m"
TEAL    = "\033[36m"
RESET   = "\033[0m"

# ################################################ Parameter
# System
SYSTEM_RATE = 30.0
SYSTEM_CYCLE = 1.0 / SYSTEM_RATE

SYSTEM_RATE_FAST = 80.0
SYSTEM_CYCLE_FAST = 1.0 / SYSTEM_RATE_FAST
# Logging path
RECORD_ACROBATIC_DIR = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/obstacle_tunnel"
RECORD_ACROBATIC_MANUEVER_NAME = "obstacle_tunnel_demo"

# Topic path
CONTROL_INPUT_TOPIC = "/on_drone/drone_control/direct/control" # Direct cause no ReactiveOA yet
CONTROL_CORRECTION_TOPIC = "/on_drone/drone_control/correction/control"  # inactive
CONTROL_FINAL_TOPIC = "/on_drone/drone_control/direct/control"  # Direct cause no ReactiveOA yet
FUSE_PERCEPTION_TOPIC = "/on_drone/sensor/fuse_perception"
CONTACT_PARSER_TOPIC = "/on_drone/sensor/body_contact"
LOGGER_RECORD_TOPIC = "/on_drone/logger/record_control"
LIDAR_2D_CONTOUR_CLOSE_TOPIC = "/on_drone/sensor/lidar2d/close/contour"
LIDAR_2D_CONTOUR_FAR_TOPIC = "/on_drone/sensor/lidar2d/far/contour"

# Service path
CONTROL_WORLD_GRASSLAND = "/world/grasslands/control"
CONTROL_WOLRD_OBSTACLE_TUNNEL = "/world/obstacle_tunnel/control"
CONTROL_WORLD_NAME = CONTROL_WOLRD_OBSTACLE_TUNNEL

# ################################################ Function
def setup_for_simulation(node):
    """
    Auto-detects if /clock exists by polling the graph.
    Does NOT create a temporary node to avoid 'Publisher already registered' warnings.
    """
    # 1. Check if the user manually set the param already. If so, respect it.
    if node.has_parameter("use_sim_time"):
        if node.get_parameter("use_sim_time").value:
            return

    # node.get_logger().info(f"{PINK}Auto check if node run in simulation...{RESET}")

    # 2. Loop Wait (Polling the Graph)
    clock_found = False
    retries = 10 # 10 * 0.1s = 1 seconds

    while retries > 0:
        # Get list of all topics currently known [('topic_name', ['type']), ...]
        topic_names_and_types = node.get_topic_names_and_types()
        
        # Extract just the names for checking
        topic_names = [t[0] for t in topic_names_and_types]

        if "/clock" in topic_names:
            clock_found = True
            break
        
        # Wait a bit for discovery
        time.sleep(0.1)
        retries -= 1

    # 3. Apply Setting
    if clock_found:
        if node.has_parameter("use_sim_time"):
            node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        else:
            node.declare_parameter("use_sim_time", True)
            
        # Pink color warning (Standard ANSI escape codes work in most terminals)
        node.get_logger().warn(f"{YELLOW}Node run using simulation clock!{RESET}")
    else:
        # Default to Realtime if not found
        node.get_logger().info(f"{PINK}No simulation clock found, run in realtime.{RESET}")
        if not node.has_parameter("use_sim_time"):
            node.declare_parameter("use_sim_time", False)
    