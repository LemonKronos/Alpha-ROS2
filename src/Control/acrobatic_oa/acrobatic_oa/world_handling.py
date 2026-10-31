"""
Handle simulation world related task: pause/play/step/reset, drone teleportation and goal points, obstacle generation.
"""
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

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
from px4_msgs.msg import VehicleCommand

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

        #: Publishers
        self.PUB_vehicle_cmd = self.the_node.create_publisher(
            VehicleCommand,
            Topic.PX4.VEHICLE_COMMAND,
            qos_profile_system_default
        )

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

        self.lastest_stats: WorldStatistics = None
        self.spawned_entities = [] # Track for clean up

    def play(self):
        req = ControlWorld.Request()
        req.world_control.pause = False
        self.SRV_control_world.call_async(req)
        self.the_node.get_logger().info(f"{GREEN} Play simulation {RESET}")

    def pause(self):
        req = ControlWorld.Request()
        req.world_control.pause = True
        self.SRV_control_world.call_async(req)
        self.the_node.get_logger().info(f"{YELLOW} Pause simulation {RESET}")

    def step(self, step: int = Physics.ONE_STEP_FAST):
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.step = True
        req.world_control.multi_step = step
        self.SRV_control_world.call_async(req)
        self.the_node.get_logger().info(f"{GREEN} Forward {step} step {RESET}")

    def reset_time(self, seed: int): #BUG 
        req = ControlWorld.Request()
        req.world_control.seed = seed
        req.world_control.reset.time_only = True
        req.world_control.pause = True
        self.SRV_control_world.call_async(req)
        self.the_node.get_logger().info(f"{YELLOW} Reset simulation time {RESET}")

    def spawn_drone(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float):
        """warning: respawn drone a second time will crash Gazebo"""
        req_spawn = SpawnEntity.Request()
        req_spawn.entity_factory.name = self.drone_id
        req_spawn.entity_factory.sdf_filename = self.drone_sdf_path
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
        req_spawn.entity_factory.pose = pose
        
        self.SRV_spawn.call_async(req_spawn)
        self.the_node.get_logger().info(f"{PINK} Spawn the drone {RESET}")

    def teleport_drone(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float):
        req_pose = SetEntityPose.Request()
        req_pose.entity.name = self.drone_id
        req_pose.entity.type = req_pose.entity.MODEL
        
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
        req_pose.pose = pose
        
        self.SRV_set_pose.call_async(req_pose)

        self.the_node.get_logger().info(
            f"{TEAL} Teleported drone to "
            f"xyz= ({x},{y},{z}), "
            f"wyxz= ({qw}, {qx},{qy},{qz}) {RESET}"
        )

    def reset_px4_ekf(self): #BUG 
        msg = VehicleCommand()
        """
        uint16 VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246 # Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.|0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.|Unused|Unused|Unused|Unused|Unused|
        """
        msg.command = VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        msg.param1 = 1.0
        msg.param2 = 0.0
    
        # Target the main system (usually 1 for SITL)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.the_node.get_clock().now().nanoseconds / 1000)

        self.PUB_vehicle_cmd.publish(msg)

        self.the_node.get_logger().info(
            f"{PINK} Sended reset PX4 EKF cmd {RESET}"
        )

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


    def spawn_entity_from_sdf(self, name: str, sdf: str, pose: Pose):
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.sdf = sdf
        req.entity_factory.pose = pose
        self.SRV_spawn.call_async(req)
        self.spawned_entities.append(name)

    def clear_spawned_entities(self):
        for entity_name in self.spawned_entities:
            req = DeleteEntity.Request()
            req.entity.name = entity_name
            req.entity.type = req.entity.MODEL
            self.SRV_delete.call_async(req)
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

    def stats_callback(self, msg: WorldStatistics):
        self.lastest_stats = msg

    def get_statistics(self) -> WorldStatistics:
        return self.lastest_stats
