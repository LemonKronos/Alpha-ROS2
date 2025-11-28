#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/surrounding.hpp"
#include "ros2_msgs/msg/control_interface.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"

// #define VISUALIZE false // uncomment to disable visualize in this node
#ifndef VISUALIZE
    #define VISUALIZE false
#endif
#ifdef VISUALIZE
    #include <visualization_msgs/msg/marker.hpp>
#endif

#include <Eigen/Dense>

using std::placeholders::_1;

constexpr uint8_t OBSTACLE_CLEAR_COUNT_THRESHOLD = 6;

class ReactiveOANode : public rclcpp::Node{
public:
    ReactiveOANode();
    ~ReactiveOANode();
private:
    // Publisher
    rclcpp::Publisher<ros2_msgs::msg::ControlInterface>::SharedPtr final_control_PUB;
    #ifdef VISUALIZE
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr movement_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr repulsive_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr correction_vec_PUB;
    #endif

    // Subscriber
    rclcpp::Subscription<ros2_msgs::msg::ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr close_contour_SUB;
    rclcpp::Subscription<ros2_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Stored data
    ros2_msgs::msg::ControlInterface::SharedPtr last_input_control = nullptr;
    ros2_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    
    // Variables
    bool last_input = true;
    bool obstacle_encountered = false;
    Obstacle obstacle;
    Eigen::Vector3f control_vec;
    Eigen::Vector3f control_angular_vec;
    Eigen::Vector3f movement_vec;
    Eigen::Vector3f movement_angular_vec;
    Eigen::Vector3f repulsive_vec;
    float repulsive_damp_unit = 0;
    Eigen::Vector3f correction_vec;
    Eigen::Vector3f correction_angular_vec;
    float safe_distance = HAZARD_DISTANCE;
    uint8_t obstacle_clear_counter = 0;

    // Methods
    void computeControlVector();
    void computeMovementVector();
    void computeRepulsiveVector();
    void computeCorrectionVector();
    void resetVectors();
    #ifdef VISUALIZE
        void publishVectorArrow(
            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
            const Eigen::Vector3f& vec,
            float r, float g, float b
        );
    #endif

    // Timer
    rclcpp::TimerBase::SharedPtr node_loop_TIME;

    // Callbacks
    void inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg);
    void closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    void perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg);
    void nodeLoopCallback();

};