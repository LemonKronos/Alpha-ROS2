#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/vector_field_histogram.hpp"

#include <bitset>

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

#define DEBUG 1
#ifndef DEBUG
    #define DEBUG 0
#endif

using std::placeholders::_1;

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
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;
    rclcpp::Subscription<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr seeing_VFH_SUB;

    // Stored data
    alpha_msgs::msg::ControlInterface::SharedPtr last_control_signal = nullptr;
    alpha_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    float hazard_distance = Drone::HAZARD_DISTANCE;

    // Variables
    bool lost_control_signal = true;
    uint8_t lost_control_signal_counter = 0;
    bool lost_perception = true;
    uint8_t lost_perception_counter = 0;
    uint8_t has_seeing_voxel_counter = 0;

    Eigen::Vector3f control_vec;
    Eigen::Vector3f movement_vec;
    Eigen::Vector3f repulsive_vec;
    Eigen::Vector3f correction_vec;
    // Eigen::Vector3f control_angular_vec;
    // Eigen::Vector3f movement_angular_vec;
    // Eigen::Vector3f repulsive_angular_vec;
    // Eigen::Vector3f correction_angular_vec;

    std::bitset<Sensor::VFH_TOTAL_BINS> VFH;

    // Methods
    void computeControlVector();
    void computeMovementVector();
    void computeVectorFieldHistogram(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg);
    void computeRepulsiveVector(const Eigen::Vector3f point);
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
    void seeingVFHCallback(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg);
    void perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
    void nodeLoopCallback();

};