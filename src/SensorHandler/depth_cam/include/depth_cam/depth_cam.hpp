#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp" 
#include "depth_cam/depth_cam_threads.hpp"

namespace alpha_brain {

#define DEBUG 1
#define TIME_ANALYSE 1

#ifndef DEBUG
    #define DEBUG 0
#endif

#ifndef TIME_ANALYSE
    #define TIME_ANALYSE 0
#endif

using std::placeholders::_1;

class DepthCamNode : public rclcpp::Node {
public:
    DepthCamNode(const rclcpp::NodeOptions& options);
    ~DepthCamNode();


private:
    // Frame transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Subscriber
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr fuse_perception_SUB;

    // Variables
    std::unique_ptr<HazardPointThread> hazard_point_thread;
    std::unique_ptr<WorldUpdateThread> world_update_thread;
    std::unique_ptr<ProcessingThread> front_processing_thread;
    std::unique_ptr<ProcessingThread> left_processing_thread;
    std::unique_ptr<ProcessingThread> right_processing_thread;

    // TODO how to bring this compile time to the threads?
    // Time analyzer
#if DEBUG && TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
#endif

    // Callbacks
    void FusePerceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);

};

} // namespace alpha_brain