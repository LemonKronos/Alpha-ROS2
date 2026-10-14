#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <shared_mutex>

#include "global_utils/alpha_brain.hpp" // Contains the MapState global_map definition
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/block.h"

namespace alpha_brain {

class MapVisualizerNode : public rclcpp::Node {
public:
    MapVisualizerNode(const rclcpp::NodeOptions& options);
    ~MapVisualizerNode();

private:
    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_marker_PUB;

    // Timer
    rclcpp::TimerBase::SharedPtr visualizer_TIME;

    // Methods
    void PublishMap();
};

} // namespace alpha_brain