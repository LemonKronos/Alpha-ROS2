"""
Handle simulation world related task: pause/play/step/reset, drone teleportation and goal points, obstacle generation.

All service calls are blocking: they wait for the ACK (or a wall-clock timeout) before returning.
They must be called from a thread other than the one spinning the node (e.g. the training thread).

Note: a service ACK from Gazebo only means "request accepted/queued", not "applied".
reset_all() therefore additionally waits until the reset is observed in WorldStatistics
(sim time back to ~0) before returning.
"""
import threading

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import numpy as np
from scipy.spatial.transform import Rotation

from ros_gz_interfaces.msg import (
    WorldStatistics,
    EntityWrench
)
from ros_gz_interfaces.srv import (
    ControlWorld,
    DeleteEntity,
    SetEntityPose,
    SpawnEntity
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose
)
from nav_msgs.msg import Odometry

from python_utils.utils import *



class WorldHandling:
    def __init__(self, the_node: Node):

        self.the_node = the_node

        #: Services client
        self.SRV_control_world = self.the_node.create_client(
            ControlWorld,
            Service.WORLD_CONTROL
        )

        self.SRV_spawn = self.the_node.create_client(
            SpawnEntity,
            Service.WORLD_SPAWN
        )
        
        self.SRV_set_pose = self.the_node.create_client(
            SetEntityPose,
            Service.WORLD_SET_POSE
        )

        self.SRV_delete = self.the_node.create_client(
            DeleteEntity,
            Service.WORLD_KILL
        )

        #: Subscribers
        self.SUB_stats = self.the_node.create_subscription(
            WorldStatistics,
            Topic.WORLD_STATS,
            self.stats_callback,
            2
        )

        self.SUB_ground_truth = self.the_node.create_subscription(
            Odometry,
            Topic.DRONE_ODOMETRY,
            self.ground_truth_callback,
            qos_profile_system_default
        )

        #: Publishers
        self.PUB_world_wrench = self.the_node.create_publisher(
            EntityWrench,
            Topic.WORLD_WRENCH,
            2
        )

        #: Data
        self.world_name = Name.Dynamic.WORLD().get()
        self.drone_id = Name.Dynamic.DRONE_ID().get()
        self.drone_sdf_path = Path.Dynamic.DRONE_MODEL_SDF().get()

        #TODO sync this with training module config 
        self.GOAL_RADIUS = 0.5

        self.SERVICE_TIMEOUT = 5.0 # Blocking service call timeout, in WALL-CLOCK seconds

        self.lastest_stats: WorldStatistics = None
        self.lastest_ground_truth: Odometry = None

        # Used to wait for a fresh WorldStatistics message (e.g. to confirm a reset was applied)
        self._stats_cond = threading.Condition()
        self._stats_count = 0

        self.spawned_entities = [] # Track for clean up

    def stats_callback(self, msg: WorldStatistics):
        with self._stats_cond:
            self.lastest_stats = msg
            self._stats_count += 1
            self._stats_cond.notify_all()

    def ground_truth_callback(self, msg: Odometry):
        self.lastest_ground_truth = msg
        
    def get_statistics(self) -> WorldStatistics:
        return self.lastest_stats

    def _blocking_service_call(self, client, req, desc: str, timeout: float = None):
        """
        Blocking service call. Sends the request, then blocks the calling thread until the
        response arrives or the timeout expires.

        Returns the response, or None on service unavailable / timeout / exception.
        The node must be spinning in another thread to deliver the response.
        """
        timeout = self.SERVICE_TIMEOUT if timeout is None else timeout
        logger = self.the_node.get_logger()

        if not client.wait_for_service(timeout_sec=timeout):
            logger.error(f"{desc}: service not available after {timeout}s")
            return None

        done = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout):
            future.cancel()
            # Avoid leaking the pending request (available in recent rclpy versions)
            remove = getattr(client, "remove_pending_request", None)
            if remove is not None:
                remove(future)
            logger.error(f"{desc}: no ACK within {timeout}s")
            return None

        if future.exception() is not None:
            logger.error(f"{desc}: service call failed: {future.exception()}")
            return None

        res = future.result()
        if hasattr(res, "success") and not res.success:
            logger.warn(f"{desc}: service replied success=False")
        return res

    def _wait_reset_applied(self, timeout: float = None, sim_time_eps: float = 0.01) -> bool:
        """
        Block until a WorldStatistics message shows sim time back at ~0, i.e. the queued
        reset has actually been executed by Gazebo. Wall-clock timeout.
        Requires at least 2 new stats messages after the call, to avoid matching a stale
        message published before the reset ran (e.g. when sim time was already 0).

        NOTE: assumes WorldStatistics.sim_time is builtin_interfaces/Time.
        Check with: ros2 interface show ros_gz_interfaces/msg/WorldStatistics
        """
        timeout = self.SERVICE_TIMEOUT if timeout is None else timeout

        with self._stats_cond:
            start = self._stats_count

            def applied():
                s = self.lastest_stats
                if s is None or self._stats_count < start + 2:
                    return False
                t = s.sim_time.sec + s.sim_time.nanosec * 1e-9
                return t < sim_time_eps

            ok = self._stats_cond.wait_for(applied, timeout)

        if not ok:
            self.the_node.get_logger().error(f"reset: sim time did not return to 0 within {timeout}s")
        return ok

    def play(self) -> bool:
        req = ControlWorld.Request()
        req.world_control.pause = False
        res = self._blocking_service_call(self.SRV_control_world, req, "play")
        ok = bool(res and res.success)
        if ok:
            self.the_node.get_logger().info(f"{GREEN} Play simulation {RESET}")
        return ok

    def pause(self) -> bool:
        req = ControlWorld.Request()
        req.world_control.pause = True
        res = self._blocking_service_call(self.SRV_control_world, req, "pause")
        ok = bool(res and res.success)
        if ok:
            self.the_node.get_logger().info(f"{YELLOW} Pause simulation {RESET}")
        return ok

    def step(self, step: int = Physics.ONE_STEP_FAST) -> bool:
        """
        Step forward, default value is one cycle fast at 100 Hz
        """
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.step = True
        req.world_control.multi_step = int(step)
        res = self._blocking_service_call(self.SRV_control_world, req, f"step x{step}")
        ok = bool(res and res.success)
        if ok:
            self.the_node.get_logger().info(f"{GREEN} Forward {step} step {RESET}")
        return ok

    def reset_all(self, seed: int) -> bool:
        req = ControlWorld.Request()
        req.world_control.seed = int(seed)
        req.world_control.reset.all = True
        req.world_control.pause = True
        res = self._blocking_service_call(self.SRV_control_world, req, "reset")
        if not (res and res.success):
            return False
        # The ACK only means the reset was queued; wait until it is actually applied.
        ok = self._wait_reset_applied()
        if ok:
            self.the_node.get_logger().info(f"{YELLOW} Reset simulation {RESET}")
        return ok

    def spawn_drone(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float) -> bool:
        """warning: respawn drone a second time will crash Gazebo"""
        req_spawn = SpawnEntity.Request()
        req_spawn.entity_factory.name = self.drone_id
        req_spawn.entity_factory.sdf_filename = self.drone_sdf_path
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
        req_spawn.entity_factory.pose = pose
        
        res = self._blocking_service_call(self.SRV_spawn, req_spawn, "spawn drone")
        ok = bool(res and res.success)
        if ok:
            self.the_node.get_logger().info(f"{PINK} Spawn the drone {RESET}")
        return ok

    def teleport_drone(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float) -> bool:
        req_pose = SetEntityPose.Request()
        req_pose.entity.name = self.drone_id
        req_pose.entity.type = req_pose.entity.MODEL
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
        req_pose.pose = pose
        
        res = self._blocking_service_call(self.SRV_set_pose, req_pose, "teleport drone")
        ok = bool(res and res.success)
        if ok:
            self.the_node.get_logger().info(
                f"{TEAL} Teleported drone to "
                f"xyz= ({x},{y},{z}), "
                f"wyxz= ({qw}, {qx},{qy},{qz}) {RESET}"
            )
        return ok

    def set_drone_wrench(self, force_x: float, force_y: float, force_z: float, torque_x: float, torque_y: float, torque_z: float):
        msg = EntityWrench()

        msg.wrench.force.x = force_x
        msg.wrench.force.y = force_y
        msg.wrench.force.z = force_z

        msg.wrench.torque.x = torque_x
        msg.wrench.torque.y = torque_y
        msg.wrench.torque.z = torque_z

        msg.entity.name = self.drone_id
        msg.entity.type = msg.entity.MODEL

        msg.header.stamp = self.the_node.get_clock().now().to_msg()

        self.PUB_world_wrench.publish(msg)

    def spawn_entity_from_sdf(self, name: str, sdf: str, pose: Pose) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.pose = pose
        res = self._blocking_service_call(self.SRV_spawn, req, f"spawn {name}")
        ok = bool(res and res.success)
        if ok:
            self.spawned_entities.append(name)  # only track what actually spawned
        return ok

    def clear_spawned_entities(self):
        for entity_name in self.spawned_entities:
            req = DeleteEntity.Request()
            req.entity.name = entity_name
            req.entity.type = req.entity.MODEL
            self._blocking_service_call(self.SRV_delete, req, f"delete {entity_name}")
        self.spawned_entities.clear()

    def show_goals(self, goal_list: list[Pose]):
        """ Spawns transparent green spheres for visual goals (no collisions) """
        self.clear_spawned_entities()

        for i, pose in enumerate(goal_list):
            name = f"visual_goal_{i}"
            sdf = f"""
            <?xml version="1.0" ?>
            <sdf version="1.8">
                <model name="{name}">
                    <static>true</static>
                    <link name="link">
                        <visual name="visual">
                            <geometry><sphere><radius>{self.GOAL_RADIUS}</radius></sphere></geometry>
                            <material>
                                <ambient>0 1 0 0.5</ambient>
                                <diffuse>0 1 0 0.5</diffuse>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
            self.spawn_entity_from_sdf(name, sdf, pose)

    def show_intermediate_points(self, intermediate_points_list: list[Pose]):
        """ Spawns transparent blue spheres for OA waypoints """
        for i, pose in enumerate(intermediate_points_list):
            name = f"visual_waypoint_{i}"
            sdf = f"""
            <?xml version="1.0" ?>
            <sdf version="1.8">
                <model name="{name}">
                    <static>true</static>
                    <link name="link">
                        <visual name="visual">
                            <geometry><sphere><radius>{self.GOAL_RADIUS}</radius></sphere></geometry>
                            <material>
                                <ambient>0 0 1 0.5</ambient>
                                <diffuse>0 0 1 0.5</diffuse>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
            self.spawn_entity_from_sdf(name, sdf, pose)

    #TODO this is just an example 
    def spaw_obstacles(self, obstacle_list: list[Pose]):
        """ Example: Spawns solid red cylindrical obstacles with collisions """
        for i, pose in enumerate(obstacle_list):
            name = f"dynamic_obstacle_{i}"
            sdf = f"""
            <?xml version="1.0" ?>
            <sdf version="1.8">
                <model name="{name}">
                    <static>true</static>
                    <link name="link">
                        <collision name="collision">
                            <geometry><cylinder><radius>0.5</radius><length>4.0</length></cylinder></geometry>
                        </collision>
                        <visual name="visual">
                            <geometry><cylinder><radius>0.5</radius><length>4.0</length></cylinder></geometry>
                            <material>
                                <ambient>1 0 0 1</ambient>
                                <diffuse>1 0 0 1</diffuse>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
            self.spawn_entity_from_sdf(name, sdf, pose)