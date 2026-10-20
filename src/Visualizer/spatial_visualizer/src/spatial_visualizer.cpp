#include "spatial_visualizer/spatial_visualizer.hpp"

spatial_container::SpatialVisualizerNode::SpatialVisualizerNode(const rclcpp::NodeOptions& options) :
    Node("low_spatial", options)
{        
        Global::setup_for_simulation(this);

        this->tsdf_layer = spatial_container::global_map.tsdf_layer;

        this->voxel_size = static_cast<float>(this->declare_parameter<double>("voxel_resolution", Sensor::VOXEL_RESOLUTION));

        perception_alive_counter = 0;
        current_position.setZero();
        frame_transform.setIdentity();

        //: Publishers
        nearby_voxels_PUB = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualizer/nearby_voxel",
            10
        );

        //: Subscribers
        perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
            Topic::FUSE_PERCEPTION,
            rclcpp::SensorDataQoS(),
            std::bind(&spatial_container::SpatialVisualizerNode::FusePerceptionCallback, this, std::placeholders::_1)
        );

        //: Timers
        loop_TIME = this->create_timer(
            std::chrono::nanoseconds(Clock::LOOP_CYCLE_NANOSEC),
            std::bind(&spatial_container::SpatialVisualizerNode::TimerCallback, this)
        );
}

spatial_container::SpatialVisualizerNode::~SpatialVisualizerNode() {}

//TODO use tf instead 
void spatial_container::SpatialVisualizerNode::FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg) {
    perception_alive_counter = Threshold::ALLOW_MISSED_TOPIC;

    if(std::isnan(msg->position[0]) || std::isnan(msg->position[1]) || std::isnan(msg->position[2])) return;
    if(std::isnan(msg->q[0]) || std::isnan(msg->q[1]) || std::isnan(msg->q[2]) || std::isnan(msg->q[3])) return;
    current_position = voxblox::Point(msg->position[0], msg->position[1], msg->position[2]);
    voxblox::Rotation current_rotation = voxblox::Rotation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    frame_transform = voxblox::Transformation(current_rotation, current_position); // Body -> World
    frame_transform = frame_transform.inverse(); // World -> Body
}

void spatial_container::SpatialVisualizerNode::TimerCallback() {
    // Check if current position is alive
    if(perception_alive_counter > 0) perception_alive_counter--;
    else {
        current_position.setZero();
        frame_transform.setIdentity();

        return;
    }

    if(nearby_voxels_PUB->get_subscription_count() == 0) return;

    // Clear storage
    nearby_voxels.clear();

    {
        // Global map mutex space
        std::shared_lock lock(spatial_container::global_map.mutex);
        if(!tsdf_layer) return;

        const float NEARBY_DISTANCE = Drone::RADIUS * 20.0f;
        const float NEARBY_DIST_SQ = NEARBY_DISTANCE * NEARBY_DISTANCE;
        
        const float BLOCK_SIZE = tsdf_layer->block_size();
        // Half-diagonal of a block for accurate spherical culling
        const float BLOCK_DIAG_HALF = std::sqrt(3.0f) * BLOCK_SIZE / 2.0f; 

        // 1. Get ONLY the blocks that currently exist in memory
        voxblox::BlockIndexList allocated_blocks;
        tsdf_layer->getAllAllocatedBlocks(&allocated_blocks);

        for(const auto& block_idx : allocated_blocks) {
            auto block_ptr = tsdf_layer->getBlockPtrByIndex(block_idx);
            if(!block_ptr) continue;

            // 2. Block-level culling: Check if the whole block is too far away
            // A block's origin is its lower corner. Add half the block size to get the true center.
            voxblox::Point block_center = block_ptr->origin() + voxblox::Point::Constant(BLOCK_SIZE / 2.0f);
            
            // If the closest possible point in this block is further than our radius, skip all 4096 voxels
            if((block_center - current_position).norm() > NEARBY_DISTANCE + BLOCK_DIAG_HALF) {
                continue;
            }

            // 3. Iterate voxels only for blocks that intersect our radius
            for(size_t linear_index = 0; linear_index < block_ptr->num_voxels(); ++linear_index) {
                const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByLinearIndex(linear_index);
                
                if(voxel.weight < Sensor::VOXEL_MIN_WEIGHT || std::abs(voxel.distance) > Sensor::VOXEL_RESOLUTION) {
                    continue;
                }

                voxblox::Point voxel_coord = block_ptr->computeCoordinatesFromLinearIndex(linear_index);
                
                // 4. Math Optimization: Use squared distance in the global map frame first
                if((voxel_coord - current_position).squaredNorm() < NEARBY_DIST_SQ) {
                    // 5. Only apply the expensive matrix transform to the points we are actually keeping
                    nearby_voxels.push_back(frame_transform * voxel_coord);
                }
            }
        }
    }

    PublishNearbyVoxels(nearby_voxels);
}

// Toggle 1 for CUBE_LIST, 0 for POINTS
#define VISUALIZE_AS_CUBES 0

void spatial_container::SpatialVisualizerNode::PublishNearbyVoxels(const std::vector<Eigen::Vector3f>& hazard_voxels) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = this->base_link.get();
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "hazard_voxels";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;

#if VISUALIZE_AS_CUBES
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.scale.x = voxel_size;
    marker.scale.y = voxel_size;
    marker.scale.z = voxel_size;
#else
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = voxel_size / 2;
    marker.scale.y = voxel_size / 2;
    marker.scale.z = voxel_size / 2;
#endif


    // Solid semi-transparent orange
    marker.color.r = 1.0f;
    marker.color.g = 0.36f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;

    // Pre-allocate memory to avoid reallocations during the loop
    marker.points.reserve(hazard_voxels.size());
    
    for(const auto& voxel : hazard_voxels) {
        geometry_msgs::msg::Point p;
        p.x = voxel.x();
        p.y = voxel.y();
        p.z = voxel.z();
        marker.points.push_back(p);
    }

    nearby_voxels_PUB->publish(marker);
}
