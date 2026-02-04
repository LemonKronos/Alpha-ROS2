#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl_conversions/pcl_conversions.h> // For converting ROS <-> PCL
#include "global_utils/system_config.hpp"

constexpr float VOXEL_RESOLUTION = 0.25f;
constexpr float MAX_RANGE = 5.0f;
constexpr float PROB_HIT = 0.7f;
constexpr float PROB_MISS =0.4f;

class OctoMapNode : public rclcpp::Node {
public:
    explicit OctoMapNode(const rclcpp::NodeOptions & options);
    ~OctoMapNode() override = default;

private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr urgent_points_PUB;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octo_map_PUB;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_points_SUB;

    // Variables
    std::shared_ptr<octomap::OcTree> ocTree;

    // Timer
    rclcpp::TimerBase::SharedPtr map_out_TIME;

    // Callbacks
    void DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void MapOutputCallback();

    // Methods
    void insertCloudEfficiently(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, const octomap::point3d& sensor_origin);

};
