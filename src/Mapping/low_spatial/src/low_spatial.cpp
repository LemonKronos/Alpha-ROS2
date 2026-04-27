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
        
        tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);

        // Put map in shared memory space
        {
            std::unique_lock lock(global_map.mutex);
            global_map.tsdf_layer = tsdf_layer;
            global_map.config = config;
        }
        
        RCLCPP_INFO(this->get_logger(), GREEN "Voxblox map ready" RESET);

        perception_alive_counter = 0;
        current_position.setZero();
        hazard_distance = Drone::HAZARD_DISTANCE;

        //_ Publishers
        memory_VFH_PUB = this->create_publisher<alpha_msgs::msg::VectorFieldHistogram>(
            Topic::VFH_HAZARD_MEMORY,
            rclcpp::SensorDataQoS()
        );

        //_ Subscribers
        perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
            Topic::FUSE_PERCEPTION,
            rclcpp::SensorDataQoS(),
            std::bind(&alpha_brain::LowSpatialNode::FusePerceptionCallback, this, std::placeholders::_1)
        );

        //_ Timers
        memory_VFH_update_TIME = this->create_timer(
            std::chrono::nanoseconds(Clock::LOOP_CYCLE_NANOSEC),
            std::bind(&alpha_brain::LowSpatialNode::UpdateMemoryVFH, this)
        );
}

alpha_brain::LowSpatialNode::~LowSpatialNode() {}

void alpha_brain::LowSpatialNode::FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg) {
    perception_alive_counter = Threshold::ALLOW_MISSED_TOPIC;
    current_position = {msg->position[0], msg->position[1], msg->position[2]};
    hazard_distance = msg->hazard_distance;
}

void alpha_brain::LowSpatialNode::UpdateMemoryVFH() {
    // Check if current position is alive
    if(perception_alive_counter > 0) perception_alive_counter--;
    else {
        RCLCPP_WARN(this->get_logger(), RED "Lost perception" RESET);
        //?! We maybe able to do a fill-execpt seeing here
        return;
    }

    std::vector<Eigen::Vector3f> hazard_voxels;
    hazard_voxels.reserve(512);

    {// Global map mutex space
        std::shared_lock lock(alpha_brain::global_map.mutex);

        if(!tsdf_layer) return;

        voxblox::Point min_coord = current_position - voxblox::Point(hazard_distance, hazard_distance, hazard_distance);
        voxblox::Point max_coord = current_position + voxblox::Point(hazard_distance, hazard_distance, hazard_distance);

        // Convert those metric coordinates instantly into block indices
        voxblox::BlockIndex min_idx = tsdf_layer->computeBlockIndexFromCoordinates(min_coord);
        voxblox::BlockIndex max_idx = tsdf_layer->computeBlockIndexFromCoordinates(max_coord);

        // Loop through the 3D grid of block indices in this tight local area
        for(int x = min_idx.x(); x <= max_idx.x(); x++) {
            for(int y = min_idx.y(); y <= max_idx.y(); y++) {
                for(int z = min_idx.z(); z <= max_idx.z(); z++) {
                    voxblox::BlockIndex block_idx(x, y, z);
                    
                    auto block_ptr = tsdf_layer->getBlockPtrByIndex(block_idx);
                    if(!block_ptr) continue;

                    for(size_t linear_index = 0; linear_index < block_ptr->num_voxels(); ++linear_index) {
                        const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByLinearIndex(linear_index);
                        if(voxel.weight < Sensor::VOXEL_MIN_WEIGHT) continue;

                        voxblox::Point voxel_coord = block_ptr->computeCoordinatesFromLinearIndex(linear_index);
                        float dist_to_drone = (voxel_coord - current_position).norm();
                        if(dist_to_drone < hazard_distance) hazard_voxels.push_back(voxel_coord);
                    }
                }
            }
        }
    }

    std::bitset<Sensor::VFH_TOTAL_BINS> VFH;
    VFH.reset();

    // Put hazard voxels into VFH
    for(const auto &point : hazard_voxels) {
        // Get scaling
        float scale_distance = std::min(1.0f, Drone::HAZARD_DISTANCE / point.z());
        float scale_angle = std::min(std::asin(scale_distance), 45*DEGREE); // #NeedTuning
        float scale_bin = static_cast<int>(std::ceil(scale_angle / Sensor::VFH_RESOLUTION));

        // Get VFH center bin
        int center_yaw_bin = static_cast<int>((point.x() + M_PI) / Sensor::VFH_RESOLUTION);
        int center_pitch_bin = static_cast<int>((point.y() + M_PI_2) / Sensor::VFH_RESOLUTION);

        // Update VFH
        for(int pitch_bin = center_pitch_bin - scale_bin; pitch_bin <= center_pitch_bin + scale_bin; pitch_bin++) {
            // Cutoff for zenith
            if(pitch_bin < 0 || pitch_bin >= Sensor::VFH_LATITUDE_BINS) continue;

            for(int y = center_yaw_bin - scale_bin; y <= center_yaw_bin + scale_bin; y++) {
                // Wrapped around for azimuth
                int yaw_bin = y % Sensor::VFH_AZIMUTH_BINS;
                while(yaw_bin < 0) yaw_bin += Sensor::VFH_AZIMUTH_BINS;

                // Get index and update
                int index = pitch_bin * Sensor::VFH_AZIMUTH_BINS + yaw_bin;
                VFH.set(index);
            }
        }
    }
    PublishMemoryVFH(VFH);
}

void alpha_brain::LowSpatialNode::PublishMemoryVFH(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH) {
    alpha_msgs::msg::VectorFieldHistogram msg;

    // Check if clear
    if(VFH.none()) return;

    // Generate payload
    memset(&msg.vfh_part, 0, sizeof(msg.vfh_part)); // Init all the bits to 0s
    for(size_t i = 0; i < Sensor::VFH_TOTAL_BINS; i++) {
        msg.vfh_part[i / Sensor::VFH_MSG_BIT_SIZE] |= (VFH[i] << (i % Sensor::VFH_MSG_BIT_SIZE));
    }

    // The rest of msg
    msg.header.frame_id = this->base_link.get();
    msg.header.stamp = this->get_clock()->now();
    this->memory_VFH_PUB->publish(msg);
}
