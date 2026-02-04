#include "octo_map/octo_map.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp> // If you need TF transforms

// Register as a component
#include <rclcpp_components/register_node_macro.hpp>

OctoMapNode::OctoMapNode(const rclcpp::NodeOptions & options)
: Node("reactive_mapper", options)
{
    // Initialize Tree
    ocTree = std::make_shared<octomap::OcTree>(VOXEL_RESOLUTION);
    ocTree->setProbHit(0.7);
    ocTree->setProbMiss(0.4);

    // Subscriber
    depth_cam_points_SUB = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        Topic::DEPTH_CAM_POINTS,
        rclcpp::SensorDataQoS(), 
        std::bind(&OctoMapNode::DepthCamCallback, this, std::placeholders::_1)
    );

    // Publisher
    urgent_points_PUB = this->create_publisher<sensor_msgs::msg::PointCloud2>(Topic::OCTO_MAP_URGENT_POINTS, rclcpp::SensorDataQoS());
    octo_map_PUB = this->create_publisher<octomap_msgs::msg::Octomap>(Topic::OCTO_MAP_URGENT_POINTS, 1);

    // Create wall timer
    map_out_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
        std::bind(&OctoMapNode::MapOutputCallback, this));
}

void OctoMapNode::DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 1. Convert ROS -> PCL
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // 2. Get Sensor Origin (Simplified: Assuming cloud is already in 'map' or 'odom' frame)
    // In reality, you'd look up TF here to get camera position.
    octomap::point3d sensor_origin(0, 0, 0); 

    // 3. Update OctoMap
    insertCloudEfficiently(pcl_cloud, sensor_origin);

    // 4. FAST OUTPUT: Extract only local occupied nodes
    // We iterate the tree, but only send points close to the drone.
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
    
    for(auto it = ocTree->begin_leafs(), end = ocTree->end_leafs(); it != end; ++it) {
        if (ocTree->isNodeOccupied(*it)) {
            // Optimization: Only add if within 3 meters of drone
            if (it.getCoordinate().norm() < 3.0) { 
                obstacle_cloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            }
        }
    }

    // 5. Publish to Reactive OA
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(obstacle_cloud, output_msg);
    output_msg.header = msg->header; // Keep timestamp synced
    urgent_points_PUB->publish(output_msg);
}

void OctoMapNode::insertCloudEfficiently(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const octomap::point3d& sensor_origin) {
    // 1. Convert PCL PointCloud -> OctoMap PointCloud
    // This is just a quick data copy, very fast.
    octomap::Pointcloud octo_cloud;
    octo_cloud.reserve(pcl_cloud.size());
    
    for (const auto& p : pcl_cloud) {
        octo_cloud.push_back(p.x, p.y, p.z);
    }

    // 2. The Built-in Magic Function
    // This handles the raycasting, max_range, and updating the tree for you.
    // Argument 3: Max Range (we use our member variable max_range_)
    // Argument 4: lazy_eval = true (Speed optimization: delays tree balancing until we need it)
    ocTree->insertPointCloud(octo_cloud, sensor_origin, MAX_RANGE, true);

    // 3. Since we used lazy_eval = true, we must manually trigger updates 
    // for the changes to take effect in the inner nodes (pruning/expanding)
    ocTree->updateInnerOccupancy();
}

void OctoMapNode::MapOutputCallback()
{
    // Serialize full map for the High-Level AI
    octomap_msgs::msg::Octomap map_msg;
    octomap_msgs::fullMapToMsg(*ocTree, map_msg);
    map_msg.header.stamp = this->now();
    map_msg.header.frame_id = "odom"; // or whatever your fixed frame is
    octo_map_PUB->publish(map_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(OctoMapNode)