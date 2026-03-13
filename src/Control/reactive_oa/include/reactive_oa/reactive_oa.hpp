#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/surrounding.hpp"
#include "ros2_msgs/msg/control_interface.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"

#define VISUALIZE true // uncomment to disable visualize in this node
#ifndef VISUALIZE
    #define VISUALIZE false
#endif
#ifdef VISUALIZE
    #include <visualization_msgs/msg/marker.hpp>
#endif
#define PUBLISH_CORRECTION_CONTROL 1

#include <Eigen/Dense>

using std::placeholders::_1;
constexpr uint8_t OBSTACLE_DAMPING_INIT = 5;
constexpr float REPULSIVE_DAMPING_CONSTANT = 0.8f;

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
    ros2_msgs::msg::ControlInterface::SharedPtr last_control_signal = nullptr;
    ros2_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    
    // Variables
    bool lost_control_signal = true;
    uint8_t lost_control_signal_counter = 0;
    bool lost_perception = true;
    uint8_t lost_perception_counter = 0;

    uint8_t obstacle_rate_mismatch_counter = 0;

    Obstacle obstacle;
    float safe_distance = Drone::HAZARD_DISTANCE;

    Eigen::Vector3f control_vec;
    Eigen::Vector3f movement_vec;
    Eigen::Vector3f repulsive_vec;
    Eigen::Vector3f correction_vec;
    // Eigen::Vector3f control_angular_vec;
    // Eigen::Vector3f movement_angular_vec;
    // Eigen::Vector3f repulsive_angular_vec;
    // Eigen::Vector3f correction_angular_vec;

    uint8_t repulsive_damping_counter = 0;
    uint8_t obstacle_clear_damping_counter = 0;

    enum ReactiveState {
        IDLING,
        ENTERING,
        RUNNING,
        LEAVING
    };
    ReactiveState reactive_state = IDLING;
    

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