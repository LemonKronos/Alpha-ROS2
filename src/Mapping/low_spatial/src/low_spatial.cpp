#include "low_spatial/low_spatial.hpp"

spatial_container::LowSpatialNode::LowSpatialNode(const rclcpp::NodeOptions& options) :
    Node("low_spatial", options)
{        
        Global::setup_for_simulation(this);

        //_ ROS2 params
        this->declare_parameters("", std::map<std::string, rclcpp::ParameterValue>{
            {"voxel_resolution", rclcpp::ParameterValue(-1.0)}, // use as kill pill
            {"truncation_distance", rclcpp::ParameterValue(Sensor::VOXEL_TRUNCATION_DISTANCE)},
            {"max_weight", rclcpp::ParameterValue(Sensor::VOXEL_MAX_WEIGHT)},
            {"voxel_carving_enabled", rclcpp::ParameterValue(true)},
            {"use_weight_dropoff", rclcpp::ParameterValue(false)},
            {"esdf_min_distance", rclcpp::ParameterValue(0.0)},
            {"esdf_max_distance", rclcpp::ParameterValue(10.0)},
            {"esdf_default_distance", rclcpp::ParameterValue(10.0)}
        });

        // Check load params
        float voxel_resolution = static_cast<float>(this->get_parameter("voxel_resolution").as_double());
        if(voxel_resolution < 0) { // kill pill
            RCLCPP_WARN(this->get_logger(), YELLOW "Config not loaded, use default" RESET);
            voxel_resolution = Sensor::VOXEL_RESOLUTION;
        }
        
        //_ Init variables
        //: TSDF
        voxblox::TsdfIntegratorBase::Config tsdf_config;
        tsdf_config.default_truncation_distance = static_cast<float>(this->get_parameter("truncation_distance").as_double());
        tsdf_config.max_weight = static_cast<float>(this->get_parameter("max_weight").as_double());
        tsdf_config.voxel_carving_enabled = this->get_parameter("voxel_carving_enabled").as_bool();
        tsdf_config.use_weight_dropoff = this->get_parameter("use_weight_dropoff").as_bool();

        //: ESDF
        voxblox::EsdfIntegrator::Config esdf_config;
        esdf_config.min_distance_m = this->get_parameter("esdf_min_distance").as_double();
        esdf_config.max_distance_m = this->get_parameter("esdf_max_distance").as_double();
        esdf_config.default_distance_m = this->get_parameter("esdf_default_distance").as_double();

        //: Init Voxblox map
        global_map.initialize(voxel_resolution, tsdf_config, esdf_config);
        this->tsdf_layer = global_map.tsdf_layer; //* just for VFH, soon to be deprecated
        
        RCLCPP_INFO(this->get_logger(), GREEN "Voxblox map ready" RESET);

        perception_alive_counter = 0;
        current_position.setZero();
        frame_transform.setIdentity();
        hazard_distance = Drone::HAZARD_DISTANCE;

        hazard_voxels.reserve(512);

        #if TIME_ANALYSE
        analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger());
        #endif

        //_ Publishers
        memory_VFH_PUB = this->create_publisher<alpha_msgs::msg::VectorFieldHistogram>(
            Topic::VFH_HAZARD_MEMORY,
            rclcpp::SensorDataQoS()
        );

        #if VISUALIZE
        hazard_voxels_PUB = this->create_publisher<alpha_msgs::msg::VoxelBlock>(
            "/visualizer/hazard_voxel",
            rclcpp::SensorDataQoS()
        );
        #endif

        //_ Subscribers
        perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
            Topic::FUSE_PERCEPTION,
            rclcpp::SensorDataQoS(),
            std::bind(&spatial_container::LowSpatialNode::FusePerceptionCallback, this, std::placeholders::_1)
        );

        //_ Timers
        memory_VFH_update_TIME = this->create_timer(
            std::chrono::nanoseconds(Clock::LOOP_CYCLE_NANOSEC),
            std::bind(&spatial_container::LowSpatialNode::UpdateMemoryVFH, this)
        );
}

spatial_container::LowSpatialNode::~LowSpatialNode() {
    #if TIME_ANALYSE
    analyzer->printSummary();
    #endif
}

void spatial_container::LowSpatialNode::FusePerceptionCallback(alpha_msgs::msg::FusePerception::SharedPtr msg) {
    perception_alive_counter = Threshold::ALLOW_MISSED_TOPIC;

    hazard_distance = msg->hazard_distance;

    if(std::isnan(msg->position[0]) || std::isnan(msg->position[1]) || std::isnan(msg->position[2])) return;
    if(std::isnan(msg->q[0]) || std::isnan(msg->q[1]) || std::isnan(msg->q[2]) || std::isnan(msg->q[3])) return;
    current_position = voxblox::Point(msg->position[0], msg->position[1], msg->position[2]);
    voxblox::Rotation current_rotation = voxblox::Rotation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

    frame_transform = voxblox::Transformation(current_rotation, current_position); // Body -> World
    frame_transform = frame_transform.inverse(); // World -> Body
}

void spatial_container::LowSpatialNode::UpdateMemoryVFH() {
    #if TIME_ANALYSE
    analyzer->start_segment("main loop");
    #endif

    //_ Build Voxblox ESDF here
    spatial_container::global_map.updateEsdf();

    // Check if current position is alive
    if(perception_alive_counter > 0) perception_alive_counter--;
    else {
        RCLCPP_WARN(this->get_logger(), RED "Lost perception, no memory VFH will be sent" RESET);
        //?! We maybe able to do a fill-execpt seeing here: not allow drone to move backward blindly

        hazard_distance = Drone::HAZARD_DISTANCE;
        current_position.setZero();
        frame_transform.setIdentity();

        return;
    }

    // Clear storage
    hazard_voxels.clear();

    {// Global map mutex space
        std::shared_lock lock(spatial_container::global_map.mutex);

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
                        if(voxel.weight < Sensor::VOXEL_MIN_WEIGHT || std::abs(voxel.distance) > Sensor::VOXEL_RESOLUTION) continue;

                        voxblox::Point voxel_coord = block_ptr->computeCoordinatesFromLinearIndex(linear_index);
                        voxel_coord = frame_transform * voxel_coord;

                        float dist_to_drone = voxel_coord.norm();
                        if(dist_to_drone < hazard_distance) hazard_voxels.push_back(voxel_coord);
                    }
                }
            }
        }
    }

    #if VISUALIZE
    PublishHazardVoxels(hazard_voxels);
    #endif

    #if FLOW
    RCLCPP_INFO(this->get_logger(), GRAY "Interfere %d points" RESET, hazard_voxels.size());
    #endif

    std::bitset<Sensor::VFH_TOTAL_BINS> VFH;
    VFH.reset();
    Eigen::Vector3f repulsive(Eigen::Vector3f::Zero());
    float max_repulsive = std::numeric_limits<float>::max();

    // Put hazard voxels into VFH
    for(auto &point : hazard_voxels) {
        // Convert to spherical
        point = math_utils::toSpherical(point);

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

        // Update repulsive
        if(max_repulsive < point.z()) {
            max_repulsive = point.z();
            repulsive = point;
        }
    }
    PublishMemoryVFH(VFH, math_utils::toCartesian(repulsive));

    #if TIME_ANALYSE
    analyzer->stop_segment("main loop");
    #endif
}

void spatial_container::LowSpatialNode::PublishMemoryVFH(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& sum_repulsive) {
    alpha_msgs::msg::VectorFieldHistogram msg;

    // Check if clear
    if(VFH.none()) return;

    // parse VFH
    memset(&msg.vfh_part, 0, sizeof(msg.vfh_part)); // Init all the bits to 0s
    for(size_t i = 0; i < Sensor::VFH_TOTAL_BINS; i++) {
        msg.vfh_part[i / Sensor::VFH_MSG_BIT_SIZE] |= (VFH[i] << (i % Sensor::VFH_MSG_BIT_SIZE));
    }

    // parse repulsive
    msg.closest_obstacle.set__x(sum_repulsive.x());
    msg.closest_obstacle.set__y(sum_repulsive.y());
    msg.closest_obstacle.set__z(sum_repulsive.z());

    // The rest of msg
    msg.header.frame_id = this->base_link.get();
    msg.header.stamp = this->get_clock()->now();
    this->memory_VFH_PUB->publish(msg);
}

#if VISUALIZE
void spatial_container::LowSpatialNode::PublishHazardVoxels(const std::vector<Eigen::Vector3f>& hazard_voxels) {
    alpha_msgs::msg::VoxelBlock msg;

    msg.header.frame_id = this->base_link.get();
    msg.header.stamp = this->get_clock()->now();
    msg.size = hazard_voxels.size();
    
    for(auto& voxel : hazard_voxels) {
        geometry_msgs::msg::Point32 v;
        v.x = voxel.x();
        v.y = voxel.y();
        v.z = voxel.z();
        msg.point_array.points.push_back(v);
    }

    hazard_voxels_PUB->publish(msg);
}
#endif
