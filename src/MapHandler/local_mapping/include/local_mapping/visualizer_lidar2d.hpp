#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_utils/global.hpp"
#include <cmath>

using std::placeholders::_1;

class Visualizer {
public:
    Visualizer(rclcpp::Node* node);
    ~Visualizer();
    void PublishLidar2dContour(const Map& map, std::string distance);
private:
    rclcpp::Node& node;
    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_close_PUB;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_far_PUB;

};