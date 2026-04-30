#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/vector_field_histogram.hpp"

#include <bitset>

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

#ifndef DO_REACTIVE_OA
    #define DO_REACTIVE_OA 1
#endif

//_ Local define
#define DEBUG (ALLOW_DEBUG & 1)
#define VISUALIZE (ALLOW_DEBUG & 1) // vectors
#define TIME_ANALYSE (ALLOW_DEBUG & 0)
#define FLOW (ALLOW_DEBUG & 0)
#define VISUAL_VFH (ALLOW_DEBUG & 1) // Total VFH

#ifdef VISUALIZE
    #include <visualization_msgs/msg/marker.hpp>
#endif

using std::placeholders::_1;

class ReactiveOANode : public rclcpp::Node{
public:
    ReactiveOANode();
    ~ReactiveOANode();
private:
    // Publisher
    rclcpp::Publisher<alpha_msgs::msg::ControlInterface>::SharedPtr final_control_PUB;
    #if VISUALIZE
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr movement_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr repulsive_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr correction_vec_PUB;
    #endif

    #if VISUAL_VFH
    rclcpp::Publisher<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr total_VFH_PUB;
    #endif

    // Subscriber
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;
    rclcpp::Subscription<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr seeing_VFH_SUB;
    rclcpp::Subscription<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr memory_VFH_SUB;

    // Objects
    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif

    // Stored data
    alpha_msgs::msg::ControlInterface::SharedPtr last_control_signal = nullptr;
    alpha_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    float hazard_distance = Drone::HAZARD_DISTANCE;

    bool lost_control_signal = true;
    uint8_t lost_control_signal_counter = 0;
    bool lost_perception = true;
    uint8_t lost_perception_counter = 0;
    uint8_t has_seeing_vfh_counter = 0;
    uint8_t has_memory_vfh_counter = 0;

    Eigen::Vector3f control_vec;
    Eigen::Vector3f movement_vec;
    Eigen::Vector3f seeing_repulsive_vec;
    Eigen::Vector3f memory_repulsive_vec;
    Eigen::Vector3f total_repulsive_vec;
    Eigen::Vector3f correction_vec;
    // Eigen::Vector3f control_angular_vec;
    // Eigen::Vector3f movement_angular_vec;
    // Eigen::Vector3f repulsive_angular_vec;
    // Eigen::Vector3f correction_angular_vec;
    
    std::bitset<Sensor::VFH_TOTAL_BINS> seeing_VFH;
    std::bitset<Sensor::VFH_TOTAL_BINS> memory_VFH;
    std::bitset<Sensor::VFH_TOTAL_BINS> total_VFH;

    // Methods
    void computeControlVector();
    void computeMovementVector();
    void computeRepulsive(Eigen::Vector3f& output_repulsive_vec, const Eigen::Vector3f& point);
    void computeCorrectionVector();
    void resetVectors();

    void parseVectorFieldHistogram(std::bitset<Sensor::VFH_TOTAL_BINS>& VFH , const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg);
    void combineVectorFieldHistogram();
    void combineRepulsiveVector();
    
    #if VISUALIZE
        Name::Dynamic::BASE_LINK base_link;
        void publishVectorArrow(
            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
            const Eigen::Vector3f& vec,
            float r, float g, float b
        );
    #endif

    #if VISUAL_VFH
    void PublishTotalVFH();
    #endif

    // Timer
    rclcpp::TimerBase::SharedPtr node_loop_TIME;

    // Callbacks
    void inputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg);
    void seeingVFHCallback(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg);
    void memoryVFHCallback(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg);
    void perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
    void nodeLoopCallback();

};