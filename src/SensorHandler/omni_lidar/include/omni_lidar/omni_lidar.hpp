#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>


#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/spatial_container.hpp"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/layer.h"
#include "voxblox/integrator/tsdf_integrator.h"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

// Local define
#define DEBUG (ALLOW_DEBUG & 1)
#define FLOW (ALLOW_DEBUG & 0)
#define VISUALIZE (ALLOW_DEBUG & 0)
#define TIME_ANALYSE (ALLOW_DEBUG & 0)

namespace spatial_container {

using std::placeholders::_1;
using std::placeholders::_2;

using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::PointCloud2ConstIterator;

class OmniLidarNode : public rclcpp::Node {
public:
    OmniLidarNode(const rclcpp::NodeOptions &options);
    ~OmniLidarNode();


private:
    // Frame transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Message Filter Subscribers
    message_filters::Subscriber<PointCloud2> lidar_down_SUB;
    message_filters::Subscriber<PointCloud2> lidar_2d_SUB;

    // Synchronizer Policy
    typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
    
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    // SHM pointer
    std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator = nullptr;

    // Custom type
    struct intergrator_batch{
        // Static
        float range_min_sq;
        float range_max_sq;

        // Dynamic
        voxblox::Transformation tranform;
        voxblox::Pointcloud pointcloud;
        voxblox::Colors colors;
    };

    // Preallocations
    intergrator_batch lidar_down_batch;
    intergrator_batch lidar_2d_batch;

    // Methods
    void ProcessBatch(
        intergrator_batch &batch,
        const PointCloud2::ConstSharedPtr &msg
    );

    // Callbacks
    void SyncedLidarCallback(
        const PointCloud2::ConstSharedPtr &msg_down,
        const PointCloud2::ConstSharedPtr &msg_2d
    );

};

} // namespace spatial_container