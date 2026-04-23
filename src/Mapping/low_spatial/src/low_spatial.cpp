#include "low_spatial/low_spatial.hpp"

alpha_brain::LowSpatialNode::LowSpatialNode(const rclcpp::NodeOptions& options) :
    Node("low_spatial", options)
{        
        Global::setup_for_simulation(this);

        // Init map
        auto tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(Sensor::VOXEL_RESOLUTION, Sensor::VOXEL_PER_SIDE);

        // Put map in shared memory space
        {
            std::unique_lock lock(global_map.mutex);
            global_map.tsdf_layer = tsdf_layer;
        }
        
        RCLCPP_INFO(this->get_logger(), GREEN "Low Spatial Map allocated" RESET);
}

alpha_brain::LowSpatialNode::~LowSpatialNode() {

}

