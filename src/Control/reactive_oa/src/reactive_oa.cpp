#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    RCLCPP_INFO(this->get_logger(), "Reactive OA Node has been started.");

    // Publisher
    final_control_PUB = this->create_publisher<ros2_msgs::msg::ControlInterface>("control/final", 10);

    // Subscriber
    input_control_SUB = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        "control/input",
        10,
        std::bind(&ReactiveOANode::inputControlCallback, this, _1));

    close_contour_SUB = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/close",
        10,
        std::bind(&ReactiveOANode::closeContourCallback, this, _1));

    perception_SUB = this->create_subscription<ros2_msgs::msg::FusePerception>(
        "/on_drone/sensor/fuse_perception", 10,
        std::bind(&ReactiveOANode::perceptionCallback, this, _1));
}
ReactiveOANode::~ReactiveOANode(){}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Input Control Callback triggered.");
}

void ReactiveOANode::closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_encountered = true;
    obstacle.topicToObstacle(msg->obstacles);
}

void ReactiveOANode::perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "Perception Callback triggered.");
}