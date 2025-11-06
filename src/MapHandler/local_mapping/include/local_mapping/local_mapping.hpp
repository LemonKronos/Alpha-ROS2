#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "global_utils/global.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "local_mapping/visualizer_lidar2d.hpp"
#include <cmath>

using std::placeholders::_1;

class LocalMappingNode : public rclcpp::Node {
public:
    LocalMappingNode();
    ~LocalMappingNode();

private:
    // Subscriber
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr obstacle_arc_close_SUB;
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr obstacle_arc_far_SUB;

    // Publisher

    // Variables
    Visualizer* vis_pointer = nullptr;
    Map instance_close_map;
    Map instance_far_map;
    Map local_map;

    // Methods
    void convertLidar2dObstacleTopicToMap(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg, Map& map);

    // Callback
    void obstacleCloseCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    void obstacleFarCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);

};