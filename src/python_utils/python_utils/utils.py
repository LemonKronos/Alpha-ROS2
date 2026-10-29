
#! MAKE SURE TO SYNC THIS WITH GLOBAL UTILS

import time
import os
import math
from rclpy.parameter import Parameter
from collections.abc import Callable

RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
PINK    = "\033[35m"
TEAL    = "\033[36m"
RESET   = "\033[0m"

DEGREE = 0.017453292

#_                                   SYSTEM CONFIG 
class Clock:
    LOOP_RATE  = 30.0 # Hz
    LOOP_CYCLE = 1.0 / LOOP_RATE
    LOOP_CYCLE_NANOSEC = LOOP_CYCLE * 1e9

    LOOP_RATE_FAST = 100.0 # Hz
    LOOP_CYCLE_FAST = 1.0 / LOOP_RATE_FAST
    LOOP_CYCLE_FAST_NANOSEC = LOOP_CYCLE_FAST * 1e9

    LOOP_RATE_SLOW = 10.0
    LOOP_CYCLE_SLOW = 1.0 / LOOP_RATE_SLOW
    LOOP_CYCLE_SLOW_NANOSEC = LOOP_CYCLE_SLOW * 1e9

    LOOP_CYCLE_HEAVY = 4.001
    LOOP_RATE_HEAVY = 1.0 / LOOP_CYCLE_HEAVY
    LOOP_CYCLE_HEAVY_NANOSEC = LOOP_CYCLE_HEAVY * 1e9


class Threshold:
    ALLOW_MISSED_TOPIC = 3
    ALLOW_MISSED_TOPIC_ns = ALLOW_MISSED_TOPIC * Clock.LOOP_CYCLE_NANOSEC

    ALLOW_MISSED_FAST_TOPIC = math.ceil(Clock.LOOP_RATE_FAST / 10.0)
    ALLOW_MISSED_FAST_TOPIC_ns = ALLOW_MISSED_FAST_TOPIC * Clock.LOOP_CYCLE_FAST_NANOSEC

    ALLOW_MISSED_NORMAL_TO_FAST_TOPIC = ALLOW_MISSED_TOPIC * ALLOW_MISSED_FAST_TOPIC


class Topic:
    RGB_CAM_FRONT = "/sensor/rgb_cam/camera/image"
    OVERVIEW_CAM = "/sensor/overview_cam/camera/image"

    CONTROL_INPUT = "/internal/drone_control/input/control"
    CONTROL_ACROBATIC = "/internal/drone_control/acrobatic/floats"
    CONTROL_REACTIVE = "/internal/drone_control/reactive/control"
    FUSE_PERCEPTION = "/internal/sensor/fuse_perception"

    LOGGER_RECORD = "/internal/logger/record_control" # Actually use to pause world

    SPATIAL_OBSERVATION = "/internal/sensor/observation_boxes"

    WORLD_STATS = "/world/stats"

    class PX4:
        VEHICLE_COMMAND = "/fmu/in/vehicle_command"

class Service:
    WORLD_CONTROL = "/world/control"
    WORLD_SPAWN = "/world/create"
    WORLD_KILL = "/world/remove"
    WORLD_SET_POSE = "/world/set_pose"


class Drone:
    WIDTH = 2.15
    LENGTH = 0.93
    HEIGHT = 0.31

    SPEED_MAX_FORWARD = 10.0
    SPEED_MAX_BACKWARD = 6.0
    SPEED_MAX_STRAFE = 10.0
    SPEED_MAX_ANGLE = math.pi/2
    SPEED_MAX_UP = 8.0
    SPEED_MAX_DOWN = 6.0
    THRUST_SAFE_LIMIT = 1.0
    HOVER_THRUST = 0.51

    # Safety
    RADIUS = 1.2
    SAFE_BUFFER = 0.1
    HAZARD_DISTANCE = RADIUS + SAFE_BUFFER


class Sensor:
    SPATIAL_BOX_DIM = 12


class Physics:
    STEP_SIZE = 0.001
    ONE_STEP_FAST = Clock.LOOP_CYCLE_FAST / STEP_SIZE


#_                                   DYNAMIC NAME
class DynamicInfo:
    """Base class for information resolved dynamically from external sources."""

    def __init__(self, logic: Callable[[], str] | None = None) -> None:
        self._update_logic = logic
        self._info: str = "UNSET"

        # Match C++ behavior: evaluate the logic during construction.
        if self._update_logic is not None:
            self.update()

    def get(self) -> str:
        """Return the current stored value."""
        return self._info

    def update(self) -> None:
        """Refresh the stored value from its source."""
        if self._update_logic is not None:
            self._info = self._update_logic()


class DynamicInfoHelper:
    """Helpers for resolving dynamic information."""

    @staticmethod
    def updateENV(env_name: str) -> str:
        """Read an environment variable, returning UNSET if missing."""
        env_info = os.getenv(env_name)

        if env_info is not None:
            return env_info

        print(f"Get ENV error {env_name}")
        return "UNSET"


class Name:
    """Dynamic and static naming utilities."""

    class Dynamic:
        class DRONE(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: DynamicInfoHelper.updateENV("DRONE_NAME")
                )

        class DRONE_ID(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: f"{Name.Dynamic.DRONE().get()}_0"
                )

        class BASE_LINK(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: f"{Name.Dynamic.DRONE_ID().get()}/base_link"
                )

        class WORLD(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: DynamicInfoHelper.updateENV("WORLD_NAME")
                )

        class RECORDED_MANUEVER(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: DynamicInfoHelper.updateENV(
                        "RECORDED_MANUEVER_NAME"
                    )
                )


class Path:
    """Dynamic and static path utilities."""

    class Dynamic:
        class RECORD_STORAGE(DynamicInfo):
            def __init__(self) -> None:
                super().__init__(
                    lambda: DynamicInfoHelper.updateENV("RECORD_STORAGE_PATH")
                )

        class DRONE_MODEL_SDF(DynamicInfo):
                    def __init__(self) -> None:
                        super().__init__(
                            lambda: DynamicInfoHelper.updateENV("DRONE_MODEL_SDF_PATH")
                        )


#_                                   FUNCTION 
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
