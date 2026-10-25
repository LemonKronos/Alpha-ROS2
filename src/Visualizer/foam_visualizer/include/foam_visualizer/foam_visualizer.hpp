#pragma once

#include <rclcpp/rclcpp.hpp>
#include "global_utils/system_config.hpp"
#include "global_utils/types.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <string>

using std::placeholders::_1;

class FoamVisualizerNode : public rclcpp::Node {
public:
    FoamVisualizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FoamVisualizerNode();

private:
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_foam_PUB;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inflated_foam_PUB;
    rclcpp::TimerBase::SharedPtr loop_TIME;

    // Variales
    std::vector<FoamSphere> foam;
    Name::Dynamic::BASE_LINK base_link;
    float smoothed_speed = 0.0f;

    void loadFoamFromParams();
    void TimerCallback();
    void perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
};
