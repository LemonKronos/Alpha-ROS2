#include <rclcpp/rclcpp.hpp>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/alpha_brain.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/vector_field_histogram.hpp"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

//_ Local define
#define DEBUG (ALLOW_DEBUG & 1)
#define FLOW (ALLOW_DEBUG & 0)
#define VISUALIZE (ALLOW_DEBUG & 1) // voxblox local map
#define TIME_ANALYSE (ALLOW_DEBUG & 1)

#if VISUALIZE
#include "alpha_msgs/msg/voxel_block.hpp"
#endif

namespace alpha_brain {

class LowSpatialNode : public rclcpp::Node {
public:
    LowSpatialNode(const rclcpp::NodeOptions& options);
    ~LowSpatialNode();

private:
    // Publishers
    rclcpp::Publisher<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr memory_VFH_PUB;

    #if VISUALIZE
    rclcpp::Publisher<alpha_msgs::msg::VoxelBlock>::SharedPtr hazard_voxels_PUB;
    #endif

    // Subscribers
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Timer
    rclcpp::TimerBase::SharedPtr memory_VFH_update_TIME;

    // Data
    Name::Dynamic::BASE_LINK base_link;

    uint8_t perception_alive_counter;
    voxblox::Point current_position;
    voxblox::Transformation frame_transform; // World -> Body
    float hazard_distance;

    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;

    // Methods
    void PublishMemoryVFH(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& sum_repulsive);

    #if VISUALIZE
    void PublishHazardVoxels(const std::vector<Eigen::Vector3f>& hazard_voxels);
    #endif

    // Callbacks
    void FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg);
    void UpdateMemoryVFH();

    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif

};

} // namespace alpha_brain
