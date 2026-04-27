#include <rclcpp/rclcpp.hpp>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/alpha_brain.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/vector_field_histogram.hpp"


namespace alpha_brain {

class LowSpatialNode : public rclcpp::Node {
public:
    LowSpatialNode(const rclcpp::NodeOptions& options);
    ~LowSpatialNode();

private:
    // Publishers
    rclcpp::Publisher<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr memory_VFH_PUB;

    // Subscribers
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Timer
    rclcpp::TimerBase::SharedPtr memory_VFH_update_TIME;

    // Data
    Name::Dynamic::BASE_LINK base_link;

    uint8_t perception_alive_counter;
    voxblox::Point current_position;
    float hazard_distance;

    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;

    // Methods
    void PublishMemoryVFH(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH);

    // Callbacks
    void FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg);
    void UpdateMemoryVFH();

};

} // namespace alpha_brain
