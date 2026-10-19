#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/spatial_container.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/spatial_observation.hpp"
#include "global_utils/types.hpp"

#include <Eigen/Dense>
#include <vector>

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

#ifndef DO_REACTIVE_OA
    #define DO_REACTIVE_OA 1
#endif

#define DEBUG (ALLOW_DEBUG & 1)
#define VISUALIZE (ALLOW_DEBUG & 1) 
#define TIME_ANALYSE (ALLOW_DEBUG & 1)

#ifdef VISUALIZE
    #include <visualization_msgs/msg/marker_array.hpp>
    #include <visualization_msgs/msg/marker.hpp>
#endif

using std::placeholders::_1;

namespace spatial_container {

class ReactiveOANode : public rclcpp::Node {
public:
    ReactiveOANode(const rclcpp::NodeOptions &options);
    ~ReactiveOANode();
private:
    // Publishers
    rclcpp::Publisher<alpha_msgs::msg::ControlInterface>::SharedPtr final_control_PUB;
    rclcpp::Publisher<alpha_msgs::msg::SpatialObservation>::SharedPtr spatial_obs_PUB;
    
    #if VISUALIZE
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr movement_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr repulsive_vec_PUB;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reactive_vec_PUB;
    #endif

    // Subscribers
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Objects
    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif

    // Stored data
    alpha_msgs::msg::ControlInterface::SharedPtr last_control_signal = nullptr;
    alpha_msgs::msg::FusePerception::SharedPtr last_perception = nullptr;
    
    bool lost_control_signal = true;
    uint8_t lost_control_signal_counter = 0;
    bool lost_perception = true;
    uint8_t lost_perception_counter = 0;

    Eigen::Vector3f control_vec;
    Eigen::Vector3f movement_vec;
    Eigen::Vector3f repulsive_vec;
    Eigen::Vector3f reactive_vec;

    std::vector<float> smoothed_close;
    Eigen::Vector3f repulsive_filter;
    std::vector<SphereCache> sphere_cache;

    std::vector<FoamSphere> multi_sphere;

    // Methods
    void computeControlVector();
    void computeMovementVector();
    void computeReactiveVector();
    void resetVectors();
    
    // Helpers
    void loadFoamFromParams();
    void extractAndPublishSpatialBoxes();
    void computeMultiSphereGradient();

    #if VISUALIZE
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
    void perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
    void nodeLoopCallback();

    // Constants for Extraction
    static constexpr float UNIT_TO_METER = 0.5f;
    static constexpr int BOX_DIM = 12;
};

} // namespace spatial_container