#include "omni_lidar/omni_lidar.hpp"

spatial_container::OmniLidarNode::OmniLidarNode(const rclcpp::NodeOptions &options) :
Node("omni_lidar_node", options)
{
    Global::setup_for_simulation(this);

    //: Frame transform
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    //: Create subcriber
    lidar_down_SUB.subscribe(this, Topic::LIDAR_1D_DOWN_PL, rmw_qos_profile_sensor_data);
    lidar_2d_SUB.subscribe(this, Topic::LIDAR_2D_AROUND_PL, rmw_qos_profile_sensor_data);

    //: Init Synchronizer
    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), lidar_down_SUB, lidar_2d_SUB);

    //: Synchronized callback
    sync->registerCallback(std::bind(&OmniLidarNode::SyncedLidarCallback, this, _1, _2));

    //: Init variable
    // for LiDAR Down
    lidar_down_batch.range_min_sq = Sensor::LIDAR_DOWN_RANGE_MIN_SQ;
    lidar_down_batch.range_max_sq = Sensor::LIDAR_DOWN_RANGE_MAX_SQ;
    lidar_down_batch.pointcloud.reserve(1);
    lidar_down_batch.colors.reserve(1);

    // for LiDAR 2D
    lidar_2d_batch.range_min_sq = Sensor::LIDAR_2D_RANGE_MIN_SQ;
    lidar_2d_batch.range_max_sq = Sensor::LIDAR_2D_RANGE_MAX_SQ;
    lidar_2d_batch.pointcloud.reserve(Sensor::LIDAR_2D_SIZE);
    lidar_2d_batch.colors.reserve(Sensor::LIDAR_2D_SIZE);

}

spatial_container::OmniLidarNode::~OmniLidarNode() {

}

void spatial_container::OmniLidarNode::ProcessBatch(
    intergrator_batch &batch,
    const PointCloud2::ConstSharedPtr &msg
) {
    batch.pointcloud.clear();
    batch.colors.clear();

    //: Get tf for LiDAR 2D
    try {
        TransformStamped tf_world = this->tf_buffer->lookupTransform(
            "world",
            msg->header.frame_id, // Current sensor's frame
            msg->header.stamp, // Time stamp of the scan
            rclcpp::Duration::from_nanoseconds(Threshold::TF_TIMEOUT_ns)
        );
        
        // Directly convert to voxblox Transformation
        const auto& translate = tf_world.transform.translation;
        const auto& rotate = tf_world.transform.rotation;
        batch.transform = voxblox::Transformation(
            voxblox::Rotation(rotate.w, rotate.x, rotate.y, rotate.z),
            voxblox::Point(translate.x, translate.y, translate.z)
        );
    }
    catch(const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(), 1000, RED "Pointcloud transform denied, cause by DYNAMIC tf: %s" RESET, ex.what());
        return;
    }

    //: Iterate through point cloud
    PointCloud2ConstIterator<float> itx(*msg, "x");
    PointCloud2ConstIterator<float> ity(*msg, "y");
    PointCloud2ConstIterator<float> itz(*msg, "z");

    //: Valid check
    for(; itx != itx.end(); ++itx, ++ity, ++itz) {
        if(!std::isfinite(*itx) || !std::isfinite(*ity) || !std::isfinite(*itz)) continue;

        float range_sq = (*itx * *itx) + (*ity * *ity) + (*itz * *itz);
        if(batch.range_min_sq <= range_sq && range_sq <= batch.range_max_sq) {
            batch.pointcloud.push_back(voxblox::Point(*itx, *ity, *itz));
            batch.colors.push_back(voxblox::Color(128, 128, 128));
        }
    }
}

void spatial_container::OmniLidarNode::SyncedLidarCallback(
    const PointCloud2::ConstSharedPtr &msg_down,
    const PointCloud2::ConstSharedPtr &msg_2d
) {
    ProcessBatch(lidar_down_batch, msg_down);

    ProcessBatch(lidar_2d_batch, msg_2d);

    //: Put both batch into Voxblox map sequentially
    {
        // Mutex lock
        std::unique_lock lock(spatial_container::global_map.mutex_tsdf);

        // Check if map exist
        if(spatial_container::global_map.tsdf_layer) {
            
            // Lazy init intergrator (once)
            if(!this->tsdf_integrator) {
                this->tsdf_integrator = std::make_unique<voxblox::FastTsdfIntegrator>(
                    spatial_container::global_map.tsdf_sparse_config,
                    spatial_container::global_map.tsdf_layer.get()
                );
                RCLCPP_INFO(this->get_logger(), GREEN "Integrator locked onto shared map" RESET);
            }

            if(!lidar_down_batch.pointcloud.empty()) {
                this->tsdf_integrator->integratePointCloud(
                    lidar_down_batch.transform,
                    lidar_down_batch.pointcloud,
                    lidar_down_batch.colors
                );
                #if FLOW
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, GREEN "LiDAR Down intergrated into shared map" RESET);
                #endif
            }

            if(!lidar_2d_batch.pointcloud.empty()) {
                this->tsdf_integrator->integratePointCloud(
                    lidar_2d_batch.transform,
                    lidar_2d_batch.pointcloud,
                    lidar_2d_batch.colors
                );
                #if FLOW
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, GREEN "LiDAR 2D intergrated into shared map" RESET);
                #endif
            }
        }
        else {
            RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(),*this->get_clock(), 1000, RED "Dropping scan, map memory not ready yet." RESET);
        }
    }
}

