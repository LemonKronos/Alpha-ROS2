import time
import os
import math
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
    LOOP_CYCLE = 0.031
    LOOP_RATE  = 1.0 / LOOP_CYCLE
    LOOP_CYCLE_NANOSEC = LOOP_CYCLE * 1e9

    LOOP_CYCLE_FAST = 0.013
    LOOP_RATE_FAST = 1.0 / LOOP_CYCLE_FAST
    LOOP_CYCLE_FAST_NANOSEC = LOOP_CYCLE_FAST * 1e9

    LOOP_CYCLE_SLOW = 1.997
    LOOP_RATE_SLOW = 1.0 / LOOP_CYCLE_SLOW
    LOOP_CYCLE_SLOW_NANOSEC = LOOP_CYCLE_SLOW * 1e9

    LOOP_CYCLE_HEAVY = 4.001
    LOOP_RATE_HEAVY = 1.0 / LOOP_CYCLE_HEAVY
    LOOP_CYCLE_HEAVY_NANOSEC = LOOP_CYCLE_HEAVY * 1e9


class Threshold:
    MISSED_TOPIC = Clock.LOOP_RATE / 10.0
    MISSED_FAST_TOPIC = Clock.LOOP_RATE_FAST / 10.0
    MISMATCH_RATE_TOPIC = math.ceil(Clock.LOOP_RATE_FAST / Clock.LOOP_RATE)


class Path:
    RECORD_ACROBATIC = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/obstacle_tunnel"
    RECORD_ACROBATIC_MANUEVER_NAME = "obstacle_tunnel_demo"


class Topic:
    DEPTH_CAM_FRONT_PL = "/sensor/depth_cam/front/points"
    DEPTH_CAM_LEFT_PL = "/sensor/depth_cam/left/points"
    DEPTH_CAM_RIGHT_PL = "/sensor/depth_cam/right/points"
    RGB_CAM_FRONT = "/sensor/rgb_cam/camera/image"
    LIDAR_2D_AROUND_SCAN = "/sensor/lidar_2d/scan"
    LIDAR_1D_DOWN_SCAN = "/sensor/lidar_1d_down/scan"
    BODY_CONTACT = "/sensor/contact_body/contact"
    ROTOR_0_CONTACT = "/sensor/contact_rotor0/contact"
    ROTOR_1_CONTACT = "/sensor/contact_rotor1/contact"
    ROTOR_2_CONTACT = "/sensor/contact_rotor2/contact"
    ROTOR_3_CONTACT = "/sensor/contact_rotor3/contact"

    CONTROL_INPUT = "/internal/drone_control/input/control"
    CONTROL_ACROBATIC = "/internal/drone_control/acrobatic/control"
    CONTROL_REACTIVE = "/internal/drone_control/reactive/control"
    FUSE_PERCEPTION = "/internal/sensor/fuse_perception"
    CONTACT_PARSER = "/internal/sensor/contacts"
    LOGGER_RECORD = "/internal/logger/record_control"
    LIDAR_2D_CONTOUR_CLOSE = "/internal/sensor/lidar2d/close/contour"
    LIDAR_2D_CONTOUR_FAR = "/internal/sensor/lidar2d/far/contour"
    VFH_HAZARD_SEEING = "/internal/mapping/hazard/seeing/vfh"
    VFH_HAZARD_MEMORY = "/internal/mapping/hazard/memory/vfh"


class Service:
    CONTROL_WORLD_GRASSLAND = "/world/grasslands/control"
    CONTROL_WOLRD_OBSTACLE_TUNNEL = "/world/obstacle_tunnel/control"
    CONTROL_WORLD_NAME = CONTROL_WOLRD_OBSTACLE_TUNNEL


class Drone:
    NAME = "alpha_minus_2_0"
    WIDTH = 2.15
    LENGTH = 0.93
    HEIGHT = 0.31

    SPEED_MAX_FORWARD = 10.0
    SPEED_MAX_BACKWARD = 10.0
    SPEED_MAX_STRAFE = 10.0
    SPEED_MAX_ANGLE = math.pi/2
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
    OCTREE_VOXEL_RESOLUTION = 0.5
    VFH_RESOLUTION = 5 * DEGREE;
    VFH_AZIMUTH_BINS = 2 * math.pi / VFH_RESOLUTION;
    VFH_LATITUDE_BINS = math.pi / VFH_RESOLUTION;
    VFH_TOTAL_BINS = VFH_AZIMUTH_BINS * VFH_LATITUDE_BINS;
    VFH_MSG_BIT_SIZE = 32;
    VFH_MSG_CHUNK_SIZE = math.ceil(VFH_TOTAL_BINS / VFH_MSG_BIT_SIZE);

# Others
WINDOW_OVERVIEW_FPV = "Alpha FPV"

# ################################################ Function

class Global:
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

    class Info:
        drone_name = "error_drone_name"
        world_name = "error_world_name"

        def __init__(self):
            self.drone_name = os.getenv('DRONE_NAME', 'error_drone_name')
            self.world_name = os.getenv('WORLD_NAME', 'error_world_name')
            
            # print(f"Loaded - World: {self.world_name}, Drone: {self.drone_name}")

        def getDroneName(self):
            return self.drone_name
        
        def getWorldName(self):
            return self.world_name