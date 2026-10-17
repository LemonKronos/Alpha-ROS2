#pragma once

#include <rclcpp/rclcpp.hpp>
#include "global_utils/system_config.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

struct FoamSphere {
    Eigen::Vector3f pos;
    float radius;
};

class FoamVisualizerNode : public rclcpp::Node {
public:
    FoamVisualizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FoamVisualizerNode();

private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foam_PUB;
    rclcpp::TimerBase::SharedPtr loop_TIME;

    // Frame transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Variales
    std::vector<FoamSphere> foam;
    Name::Dynamic::BASE_LINK base_link;

    void loadFoamFromParams();
    void TimerCallback();
};
