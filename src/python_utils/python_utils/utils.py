import time
from rclpy.parameter import Parameter

RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
PINK    = "\033[35m"
TEAL    = "\033[36m"
RESET   = "\033[0m"

DEGREE = 0.017453292

# ################################################ Parameter
class Clock:
    LOOP_RATE = 30.0
    LOOP_CYCLE = 1.0 / LOOP_RATE

    LOOP_RATE_FAST = 80.0
    LOOP_CYCLE_FAST = 1.0 / LOOP_RATE_FAST


class Path:
    RECORD_ACROBATIC = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/obstacle_tunnel"
    RECORD_ACROBATIC_MANUEVER_NAME = "obstacle_tunnel_demo"


class Topic:
    CONTROL_INPUT = "/on_drone/drone_control/input/control" # Raw human control signal
    CONTROL_ACROBATIC = "/on_drone/drone_control/acrobatic/control" # Control signal after pass through AcrobaticOA brain
    CONTROL_REACTIVE = "/on_drone/drone_control/reactive/control" # Control signal after pass through ReactiveOA
    FUSE_PERCEPTION = "/on_drone/sensor/fuse_perception"
    CONTACT_PARSER = "/on_drone/sensor/body_contact"
    LOGGER_RECORD = "/on_drone/logger/record_control"
    LIDAR_2D_CONTOUR_CLOSE = "/on_drone/sensor/lidar2d/close/contour"
    LIDAR_2D_CONTOUR_FAR = "/on_drone/sensor/lidar2d/far/contour"
    DEPTH_CAM = "sensor/depth_cam/camera/image" 


class Service:
    CONTROL_WORLD_GRASSLAND = "/world/grasslands/control"
    CONTROL_WOLRD_OBSTACLE_TUNNEL = "/world/obstacle_tunnel/control"
    CONTROL_WORLD_NAME = CONTROL_WOLRD_OBSTACLE_TUNNEL


class Drone:
    NAME = "alpha_minus_2_0"
    WIDTH = 2.144
    LENGTH = 0.55
    HEIGHT = 0.05

    SPEED_MAX_FORWARD = 10.0
    SPEED_MAX_BACKWARD = 10.0
    SPEED_MAX_STRAFE = 10.0
    SPEED_MAX_UP = 8.0
    SPEED_MAX_DOWN = 4.0
    THRUST_SAFE_LIMIT = 0.9
    HOVER_THRUST = -0.51

    # Safety
    RADIUS = 1.2
    UNCERTAINTY = 0.05
    SAFE_BUFFER = 0.15
    HAZARD_DISTANCE = RADIUS + SAFE_BUFFER + UNCERTAINTY
    REACT_TIME = Clock.LOOP_CYCLE_FAST # s
    DECELERATE_MAX = 4.0 # m/s^2

class Sensor:
    LIDAR_2D_SECTOR_NUM = 12
    LIDAR_2D_RANGE_MAX = 30.0
    LIDAR_2D_RANGE_MIN = 0.1
    DEPTH_MAX_DIST = 30.0

# Others
WINDOW_OVERVIEW_FPV = "Alpha FPV"

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
    retries = 30 # 3 seconds

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
    