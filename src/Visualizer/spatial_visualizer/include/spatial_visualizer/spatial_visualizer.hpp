#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/spatial_container.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"
#include "alpha_msgs/msg/voxel_block.hpp"

namespace spatial_container {

class SpatialVisualizerNode : public rclcpp::Node {
public:
    SpatialVisualizerNode(const rclcpp::NodeOptions& options);
    ~SpatialVisualizerNode();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearby_voxels_PUB;

    // Subscribers
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_SUB;

    // Timer
    rclcpp::TimerBase::SharedPtr loop_TIME;

    // Data
    Name::Dynamic::BASE_LINK base_link;

    uint8_t perception_alive_counter;
    voxblox::Point current_position;
    voxblox::Transformation frame_transform; // World -> Body

    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;

    // Persistent allocation
    std::vector<Eigen::Vector3f> nearby_voxels;

    // Methods
    void PublishNearbyVoxels(const std::vector<Eigen::Vector3f>& nearby_voxels);

    // Callbacks
    void FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg);
    void TimerCallback();
};

} // namespace spatial_container
