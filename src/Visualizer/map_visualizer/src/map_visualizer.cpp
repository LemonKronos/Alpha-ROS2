#include "map_visualizer/map_visualizer.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

spatial_container::MapVisualizerNode::MapVisualizerNode(const rclcpp::NodeOptions &options) :
Node("map_visualizer_node", options)
{
    // Create publisher
    map_marker_PUB = this->create_publisher<visualization_msgs::msg::Marker>(
        "tsdf_map_marker", 
        rclcpp::QoS(1)
    );

    // Create a timer to publish the map at 2 Hz
    visualizer_TIME = this->create_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapVisualizerNode::PublishMap, this)
    );

    RCLCPP_INFO(this->get_logger(), "Map Visualizer initialized.");
}

spatial_container::MapVisualizerNode::~MapVisualizerNode() {
}

void spatial_container::MapVisualizerNode::PublishMap() {
    // Only proceed if there are subscribers to save CPU
    if (map_marker_PUB->get_subscription_count() == 0) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "voxblox_surface";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // Shared read lock to prevent modifying the map while we iterate, 
    // but allowing multiple readers (like Reactive OA) to access it simultaneously.
    std::shared_lock<std::shared_mutex> lock(global_map.mutex_tsdf);

    if (!global_map.tsdf_layer) {
        return; // Map not initialized yet
    }

    // Set scale based on the actual voxel size
    float voxel_size = global_map.tsdf_layer->voxel_size();
    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size;
    marker.scale.z = voxel_size;

    // Extract blocks
    voxblox::BlockIndexList blocks;
    global_map.tsdf_layer->getAllAllocatedBlocks(&blocks);

    for (const voxblox::BlockIndex& index : blocks) {
        voxblox::Block<voxblox::TsdfVoxel>::ConstPtr block = global_map.tsdf_layer->getBlockPtrByIndex(index);
        
        for (size_t linear_index = 0; linear_index < block->num_voxels(); ++linear_index) {
            const voxblox::TsdfVoxel& voxel = block->getVoxelByLinearIndex(linear_index);
            
            // Filter unobserved voxels
            if (voxel.weight > 1e-3) {
                // Only visualize the surface (where distance is close to 0) to keep RViz fast
                if (std::abs(voxel.distance) < voxel_size) {
                    voxblox::Point coord = block->computeCoordinatesFromLinearIndex(linear_index);
                    
                    geometry_msgs::msg::Point pt;
                    pt.x = coord.x();
                    pt.y = coord.y();
                    pt.z = coord.z();
                    marker.points.push_back(pt);

                    // Color based on height or use the stored voxel color
                    std_msgs::msg::ColorRGBA color;
                    color.r = voxel.color.r / 255.0f;
                    color.g = voxel.color.g / 255.0f;
                    color.b = voxel.color.b / 255.0f;
                    color.a = 0.8f; 
                    marker.colors.push_back(color);
                }
            }
        }
    }

    // Unlock before publishing to minimize lock contention
    lock.unlock();

    map_marker_PUB->publish(marker);
}