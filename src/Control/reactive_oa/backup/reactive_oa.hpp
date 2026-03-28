#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/surrounding.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/lidar2d_obstacle.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/voxel_block.hpp"

#define VISUALIZE true // uncomment to disable visualize in this node
#ifndef VISUALIZE
    #define VISUALIZE false
#endif
#ifdef VISUALIZE
    #include <visualization_msgs/msg/marker.hpp>
#endif

#ifndef DO_REACTIVE_OA
    #define DO_REACTIVE_OA 1
#endif

using std::placeholders::_1;
constexpr uint8_t HAS_SEEING_VOXEL_COUNTER_INIT = 3;
constexpr uint8_t OBSTACLE_DAMPING_INIT = 5;
constexpr float REPULSIVE_DAMPING_CONSTANT = 0.8f;

class ReactiveOANode : public rclcpp::Node{
public:
    ReactiveOANode();
    ~ReactiveOANode();
private:
    // Publisher
    rclcpp::Publisher<alpha_msgs::msg::ControlInterface>::SharedPtr final_control_PUB;
    #ifdef VISUALIZE
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr movement_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr repulsive_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr correction_vec_PUB;
    #endif

    // Subscriber
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<alpha_msgs::msg::Lidar2dObstacle>::SharedPtr close_contour_SUB;
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;
    rclcpp::Subscription<alpha_msgs::msg::VoxelBlock>::SharedPtr seeing_voxel_SUB;

    // Stored data
    alpha_msgs::msg::ControlInterface::SharedPtr last_control_signal = nullptr;
    alpha_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    alpha_msgs::msg::VoxelBlock::SharedPtr last_seeing_voxel = nullptr;
    
    // Variables
    bool lost_control_signal = true;
    uint8_t lost_control_signal_counter = 0;
    bool lost_perception = true;
    uint8_t lost_perception_counter = 0;
    uint8_t has_seeing_voxel_counter = 0;

    uint8_t obstacle_rate_mismatch_counter = 0;

    Obstacle obstacle;
    float safe_distance = Drone::HAZARD_DISTANCE;
    float seeing_voxel_safe_distance = Drone::HAZARD_DISTANCE;

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

    // Methods
    void computeControlVector();
    void computeMovementVector();
    void computeRepulsiveVector();
    void computeCorrectionVector();
    void resetVectors();
    
    #ifdef VISUALIZE
        Name::Dynamic::BASE_LINK base_link;
        void publishVectorArrow(
            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
            const Eigen::Vector3f& vec,
            float r, float g, float b
        );
    #endif

    // Timer
    rclcpp::TimerBase::SharedPtr node_loop_TIME;

    // Callbacks
    void inputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg);
    void closeContourCallback(const alpha_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    void perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
    void nodeLoopCallback();

};