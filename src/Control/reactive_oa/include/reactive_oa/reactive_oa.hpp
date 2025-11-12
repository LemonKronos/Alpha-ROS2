#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/surrounding.hpp"
#include "ros2_msgs/msg/control_interface.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"

using std::placeholders::_1;

class ReactiveOANode : public rclcpp::Node{
public:
    ReactiveOANode();
    ~ReactiveOANode();
private:
    // Publisher
    rclcpp::Publisher<ros2_msgs::msg::ControlInterface>::SharedPtr final_control_PUB;

    // Subscriber
    rclcpp::Subscription<ros2_msgs::msg::ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr close_contour_SUB;
    rclcpp::Subscription<ros2_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Variables
    bool obstacle_encountered = false;
    Obstacle obstacle;

    // Callbacks
    void inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg);
    void closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    void perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg);


};