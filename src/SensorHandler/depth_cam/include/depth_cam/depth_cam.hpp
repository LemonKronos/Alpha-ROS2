#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "depth_cam/depth_cam_threads.hpp"

namespace spatial_container {

using std::placeholders::_1;

class DepthCamNode : public rclcpp::Node {
public:
    DepthCamNode(const rclcpp::NodeOptions& options);
    ~DepthCamNode();

private:
    // Frame transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Variables
    std::unique_ptr<WorldUpdateThread> world_update_thread;
    std::unique_ptr<ProcessingThread> front_processing_thread;
    std::unique_ptr<ProcessingThread> left_processing_thread;
    std::unique_ptr<ProcessingThread> right_processing_thread;

    // Time analyzer
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
};

} // namespace spatial_container