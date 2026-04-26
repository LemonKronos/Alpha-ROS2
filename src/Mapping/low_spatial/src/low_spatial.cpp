#include "low_spatial/low_spatial.hpp"

alpha_brain::LowSpatialNode::LowSpatialNode(const rclcpp::NodeOptions& options) :
    Node("low_spatial", options)
{        
        Global::setup_for_simulation(this);

        //_ ROS2 params
        this->declare_parameters("", std::map<std::string, rclcpp::ParameterValue>{
            {"voxel_resolution", rclcpp::ParameterValue(-1.0)}, // use as kill pill
            {"truncation_distance", rclcpp::ParameterValue(Sensor::VOXEL_TRUNCATION_DISTANCE)},
            {"max_weight", rclcpp::ParameterValue(Sensor::VOXEL_MAX_WEIGHT)}
        });

        float voxel_resolution = static_cast<float>(this->get_parameter("voxel_resolution").as_double());
        if(voxel_resolution < 0) { // Check kill pill
            RCLCPP_WARN(this->get_logger(), YELLOW "Config not loaded, use default" RESET);
            voxel_resolution = Sensor::VOXEL_RESOLUTION;
        }
        
        //_ Init variables
        // Global map
        voxblox::TsdfIntegratorBase::Config config;
        config.default_truncation_distance = static_cast<float>(this->get_parameter("truncation_distance").as_double());
        config.max_weight = static_cast<float>(this->get_parameter("max_weight").as_double());
        config.voxel_carving_enabled = true;
        
        auto tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);

        // Put map in shared memory space
        {
            std::unique_lock lock(global_map.mutex);
            global_map.tsdf_layer = tsdf_layer;
            global_map.config = config;
        }
        
        RCLCPP_INFO(this->get_logger(), GREEN "Voxblox map ready" RESET);
}

alpha_brain::LowSpatialNode::~LowSpatialNode() {

}

