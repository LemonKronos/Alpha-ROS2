#include "depth_cam/depth_cam_threads.hpp"

#pragma region ProcessingThread class

alpha_brain::ProcessingThread::ProcessingThread(
    const std::string& name,
    rclcpp::Node* theNode,
    const std::string& topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>>& hazard_point_queue,
    const std::atomic<bool>& world_update,
    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue,
    time_utils::TimeAnalyzer* analyzer
) : 
    name(name), 
    theNode(theNode), 
    topic(topic), 
    tf_buffer(tf_buffer), 
    world_update(world_update), 
    hazard_point_queue(hazard_point_queue), 
    world_update_queue(world_update_queue),
    analyzer(analyzer)
{
    // Create subscriber
    this->depth_cam_SUB = theNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->topic,
        rclcpp::SensorDataQoS(),
        std::bind(&alpha_brain::ProcessingThread::DepthCamCallback, this, _1)
    );

    // Init variables
    this->has_tf_body = false;
    this->running.store(true);
    this->hazard_distance.store(Drone::HAZARD_DISTANCE);
    this->done_world_update = false;

    // Spawn thread
    this->processing_thread = std::thread(&ProcessingThread::ConsumerLoop, this);

#if FLOW
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn worker %s processing thread" RESET, this->name.c_str());
#endif
}

alpha_brain::ProcessingThread::~ProcessingThread() {
    this->running.store(false);
    this->msg_queue.enqueue(nullptr);
    if(this->processing_thread.joinable()) this->processing_thread.join();

#if FLOW
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "%s processing thread destructor called" RESET, this->name.c_str());
#endif
}

void alpha_brain::ProcessingThread::processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if(!this->has_tf_body) {
        try {
            geometry_msgs::msg::TransformStamped tf_body = this->tf_buffer->lookupTransform(
                this->base_link.get(), // Target frame: body
                msg->header.frame_id, // Current frame: depth camera
                rclcpp::Time(0) // Time stamp don't matter
            );
            this->iso_body = tf2::transformToEigen(tf_body).cast<float>();
            this->has_tf_body = true;
            RCLCPP_INFO(this->theNode->get_logger(), GREEN "%s depth camera STATIC tf lookup complete" RESET, this->name.c_str());
        }
        catch(const tf2::TransformException& ex) {
            RCLCPP_WARN(this->theNode->get_logger(), RED "%s point clouds msg denied, cause by STATIC tf: %s" RESET, this->name.c_str(), ex.what());
            return;
        }
    }
    this->msg_queue.enqueue(msg);
}

void alpha_brain::ProcessingThread::DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    processMsg(msg);
}

void alpha_brain::ProcessingThread::updateSafeBubble(const float hazard_distance) {
    this->hazard_distance.store(hazard_distance);
}

void alpha_brain::ProcessingThread::ConsumerLoop() {
    
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the msg
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        this->msg_queue.wait_dequeue(msg);
        while(this->msg_queue.try_dequeue(msg)) {
            RCLCPP_WARN(this->theNode->get_logger(), YELLOW "%s flushed a mesage" RESET, this->name.c_str());
        } // FLush to only use latest msg
        if(!this->running.load(std::memory_order_relaxed)) break;

        // Load local atomic variables
        bool world_update = this->world_update.load(std::memory_order_relaxed);
        float hazard_distance_sq = this->hazard_distance.load(std::memory_order_relaxed); // Extract hazard_distance
        hazard_distance_sq *= hazard_distance_sq; // Square it

        // Check to make sure do world update only once per call
        if(!world_update) this->done_world_update = false;

        // Lookup world frame
        std::optional<voxblox::Transformation> voxblox_tf_world;
        if(world_update && !this->done_world_update) {
#if FLOW
            RCLCPP_INFO(this->theNode->get_logger(), YELLOW "%s thread send world update" RESET, this->name.c_str());
#endif            
            try {
                geometry_msgs::msg::TransformStamped tf_world = this->tf_buffer->lookupTransform(
                    "world",
                    msg->header.frame_id, // Current frame: depth camera
                    msg->header.stamp, // Time stamp of the scan
                    rclcpp::Duration::from_nanoseconds(Clock::LOOP_CYCLE_NANOSEC * 2)
                );
                
                // Directly convert to voxblox Transformation
                const auto& translate = tf_world.transform.translation;
                const auto& rotate = tf_world.transform.rotation;
                voxblox_tf_world = voxblox::Transformation(
                    voxblox::Rotation(rotate.w, rotate.x, rotate.y, rotate.z),
                    voxblox::Point(translate.x, translate.y, translate.z)
                );
            }
            catch(const tf2::TransformException& ex) {
                RCLCPP_WARN(this->theNode->get_logger(), RED "%s transform denied, cause by DYNAMIC tf: %s" RESET, this->name.c_str(), ex.what());
            }
        }

        // Prepare intermediate variables>
        bool hazard_exist = false;
        std::vector<Eigen::Vector3f> hazard_cloud;

        VoxbloxBatch world_update_cloud;
        if(world_update && !this->done_world_update && voxblox_tf_world.has_value()) {
            world_update_cloud.points.reserve(WORLD_BATCH_SIZE);
            world_update_cloud.transfrom = voxblox_tf_world.value();
        }

        sensor_msgs::PointCloud2Iterator<float> itx(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> ity(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> itz(*msg, "z");

        // Iterate through point cloud
        for(; itx != itx.end(); ++itx, ++ity, ++itz) {
            if(!std::isfinite(*itx) || !std::isfinite(*ity) || !std::isfinite(*itz)) continue;

            Eigen::Vector3f raw_point(*itx, *ity, *itz); // Raw from depth camera
            Eigen::Vector3f body_point = this->iso_body * raw_point; // Transformed to body frame
            
            // Body clipping check
            bool x_cliped = (Drone::MIN_X < body_point.x() && body_point.x() < Drone::MAX_X);
            bool y_cliped = (Drone::MIN_Y < body_point.y() && body_point.y() < Drone::MAX_Y);
            bool z_cliped = (Drone::MIN_Z < body_point.z() && body_point.z() < Drone::MAX_Z);
            if(x_cliped && y_cliped && z_cliped) continue;

            //_ For hazard point
            // hazard_distance_sq = 400.0f; //#TEST
            if(body_point.squaredNorm() <= hazard_distance_sq) {
                // Convert to spherical coordinate
                Eigen::Vector3f spherical_body_point = math_utils::toSpherical(body_point);

                // Push into batch
                if(hazard_exist) {
                    if(hazard_cloud.size() >= HAZARD_BATCH_SIZE) {
                        this->hazard_point_queue.enqueue(std::move(hazard_cloud));
                        hazard_cloud = std::vector<Eigen::Vector3f>(); // #CanBeOptimize
                        hazard_cloud.reserve(HAZARD_BATCH_SIZE);
                    }
                }
                else {
                    hazard_cloud = std::vector<Eigen::Vector3f>(); // #CanBeOptimize
                    hazard_cloud.reserve(HAZARD_BATCH_SIZE);
                    hazard_exist = true;
                }
                hazard_cloud.push_back(spherical_body_point);
            }

            //_ For world update
            if(world_update && !this->done_world_update && voxblox_tf_world.has_value()) {
                if(world_update_cloud.points.size() >= WORLD_BATCH_SIZE) {
                    this->world_update_queue.enqueue(std::move(world_update_cloud));
                    world_update_cloud = VoxbloxBatch(); // #CanBeOptimize
                    world_update_cloud.points.reserve(WORLD_BATCH_SIZE);
                    world_update_cloud.transfrom = voxblox_tf_world.value();
                }
                world_update_cloud.points.push_back(raw_point.cast<voxblox::FloatingPoint>());
            }
        }

        // Flush the left over batch and Send empty batch to indicate end of msg
        if(hazard_cloud.size() > 0) this->hazard_point_queue.enqueue(std::move(hazard_cloud));
        this->hazard_point_queue.enqueue(std::vector<Eigen::Vector3f>());
        if(world_update && !this->done_world_update) {
            if(world_update_cloud.points.size() > 0) this->world_update_queue.enqueue(std::move(world_update_cloud));
            this->world_update_queue.enqueue(alpha_brain::VoxbloxBatch());
            this->done_world_update = true;
        }
    }
}

#pragma endregion

#pragma region HazardPointThread class

alpha_brain::HazardPointThread::HazardPointThread(
    rclcpp::Node* theNode,
    const int num_worker
) : 
    theNode(theNode), 
    num_worker(num_worker) 
{
    // Create Publisher
    this->seeing_VFH_PUB = this->theNode->create_publisher<alpha_msgs::msg::VectorFieldHistogram>(
        Topic::VFH_HAZARD_SEEING,
        rclcpp::SensorDataQoS()
    );

    // Init variables
    this->running.store(true);

    // Check sycn
    alpha_msgs::msg::VectorFieldHistogram test_msg;
    if(test_msg.vfh_part.size() != Sensor::VFH_MSG_CHUNK_SIZE) {
        std::string error_msg = 
              "Wrong VFH msg size: expected "
            + std::to_string(Sensor::VFH_MSG_CHUNK_SIZE)
            + ", but got "
            + std::to_string(test_msg.vfh_part.size());
        RCLCPP_FATAL(this->theNode->get_logger(), RED "%s" RESET, error_msg.c_str());
        throw std::runtime_error(error_msg);
    }

    // Spawn persistent thread
    this->hazard_point_thread = std::thread(&HazardPointThread::ConsumerLoop, this);

#if FLOW    
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn Consumer thread Hazard Point" RESET);
#endif    
}

alpha_brain::HazardPointThread::~HazardPointThread() {
    this->running.store(false);
    this->hazard_point_queue.enqueue(std::vector<Eigen::Vector3f>());
    if(this->hazard_point_thread.joinable()) this->hazard_point_thread.join();

#if FLOW    
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "Hazard point thread destructor called" RESET);
#endif    

#if TIME_ANALYSE
        theNode->analyzer->printSummary();
#endif
}

moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>>& alpha_brain::HazardPointThread::getQueue() {
    return this->hazard_point_queue;
}

void alpha_brain::HazardPointThread::ConsumerLoop() {
    int worker_finished = 0;
    std::bitset<Sensor::VFH_TOTAL_BINS> VFH;
    VFH.reset();
    Eigen::Vector3f repulsive_direction(Eigen::Vector3f::Zero());
    float repulsive_value = std::numeric_limits<float>::max();

    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        std::vector<Eigen::Vector3f> batch_cloud;
        this->hazard_point_queue.wait_dequeue(batch_cloud);

#if DEBUG && TIME_ANALYSE
        // theNode->analyzer->start_segment("Collect Batch Cloud");
#endif

        if(!this->running.load(std::memory_order_relaxed)) break;

        if(batch_cloud.empty()) {
            worker_finished++;
            if(worker_finished >= this->num_worker) {
                PublishHazardPoint(VFH, math_utils::toCartesian(repulsive_direction));
                VFH.reset();
                repulsive_direction.setZero();
                repulsive_value = FLT_MAX;
                worker_finished = 0;
            }
            continue;
        }

        // Put the batch cloud to VFH 
        // #CanBeOptimize maybe try do the repulsive update in here
        for(const auto &point : batch_cloud) {
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

            // Update closest point
            if(repulsive_value > point.z()) {
                repulsive_value = point.z();
                repulsive_direction = point;
            }
        }

#if DEBUG && TIME_ANALYSE
        // theNode->analyzer->stop_segment("Collect Batch Cloud");
#endif  

    }
}

void alpha_brain::HazardPointThread::PublishHazardPoint(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& sum_repulsive) {
    alpha_msgs::msg::VectorFieldHistogram msg;

    // Check if clear
    if(VFH.none()) return;

    // Generate payload
    memset(&msg.vfh_part, 0, sizeof(msg.vfh_part)); // Init all the bits to 0s
    for(size_t i = 0; i < Sensor::VFH_TOTAL_BINS; i++) {
        msg.vfh_part[i / Sensor::VFH_MSG_BIT_SIZE] |= (VFH[i] << (i % Sensor::VFH_MSG_BIT_SIZE));
    }

    // Generate closest point
    msg.closest_obstacle.set__x(sum_repulsive.x());
    msg.closest_obstacle.set__y(sum_repulsive.y());
    msg.closest_obstacle.set__z(sum_repulsive.z());

    // The rest of msg
    msg.header.frame_id = this->base_link.get();
    msg.header.stamp = this->theNode->get_clock()->now();
    this->seeing_VFH_PUB->publish(msg);
}

#pragma endregion

#pragma region WorlUpdateThread class

alpha_brain::WorldUpdateThread::WorldUpdateThread(
    rclcpp::Node* theNode,
    const int num_worker,
    time_utils::TimeAnalyzer* analyzer
) : 
    theNode(theNode), 
    num_worker(num_worker),
    analyzer(analyzer)
{
    // Init variables
    this->running.store(true);
    this->world_update.store(false);
    
    // Create wall timer
    world_update_TIME = this->theNode->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
        std::bind(&alpha_brain::WorldUpdateThread::doWorldUpdate, this)
    );

    // Spawn persistent thread
    this->world_update_thread = std::thread(&WorldUpdateThread::ConsumerLoop, this);

    #if FLOW
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn Consumer thread World Update" RESET);
    #endif
}

alpha_brain::WorldUpdateThread::~WorldUpdateThread() {
    this->running.store(false);
    this->world_update_queue.enqueue(alpha_brain::VoxbloxBatch());
    if(this->world_update_thread.joinable()) this->world_update_thread.join();

    #if FLOW    
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "World update thread destructor called" RESET);
    #endif    
}

const std::atomic<bool>& alpha_brain::WorldUpdateThread::getStatus() {
    return this->world_update;
}

moodycamel::BlockingConcurrentQueue<alpha_brain::VoxbloxBatch>& alpha_brain::WorldUpdateThread::getQueue() {
    return this->world_update_queue;
}

void alpha_brain::WorldUpdateThread::doWorldUpdate() {
    this->world_update.store(true);

    #if TIME_ANALYSE
    this->analyzer->start_segment("World update");
    #endif

    #if FLOW    
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Start world update" RESET);
    #endif    
}

void alpha_brain::WorldUpdateThread::ConsumerLoop() {
    uint8_t worker_finished = 0;
    #if FLOW
    bool has_data = false;
    int count = 0;
    #endif

    // Thread loop
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        VoxbloxBatch batch_cloud;
        this->world_update_queue.wait_dequeue(batch_cloud);

        // Check stop thread condition
        if(!this->running.load(std::memory_order_relaxed)) break;

        // Flush batch if not currently in world_update
        if(!this->world_update.load(std::memory_order_relaxed)) {
            #if FLOW            
            RCLCPP_INFO(this->theNode->get_logger(), GRAY "Flush %d points" RESET, batch_cloud.points.size());
            #endif            
            continue;
        }

        // Receive valid batch
        #if FLOW
        RCLCPP_INFO(this->theNode->get_logger(), GRAY "Receive batch with %d points" RESET, batch_cloud.points.size());
        count += batch_cloud.points.size();
        #endif        

        if(batch_cloud.points.empty()) {
            worker_finished++;
            #if FLOW            
            RCLCPP_INFO(this->theNode->get_logger(), GRAY "%d worker_finished, got %d points" RESET, worker_finished, count);
            #endif

            // Received all batch
            if(worker_finished >= this->num_worker) {
                this->world_update.store(false);

                #if TIME_ANALYSE
                this->analyzer->stop_segment("World update");
                #endif

                #if FLOW    
                RCLCPP_INFO(this->theNode->get_logger(), GRAY "Added %d points" RESET, count);
                if(has_data && worker_finished == this->num_worker) RCLCPP_INFO(this->theNode->get_logger(), GREEN "Wolrd update complete" RESET);
                else if(has_data && worker_finished > this->num_worker) RCLCPP_WARN(this->theNode->get_logger(), YELLOW "Wolrd update overloaded" RESET);
                else RCLCPP_INFO(this->theNode->get_logger(), GREEN "Wolrd update empty" RESET);
                count = 0;
                has_data = false;
                #endif

                worker_finished = 0;
            }
            else continue;
        }

        #if FLOW
        has_data = true;
        #endif        

        // Put new scan to voxblox map
        {
            // Mutex lock
            std::unique_lock lock(alpha_brain::global_map.mutex);

            // Check if map exist
            if(alpha_brain::global_map.tsdf_layer) {
                
                // Lazy init intergrator (once)
                if(!this->tsdf_integrator) {
                    this->tsdf_integrator = std::make_unique<voxblox::FastTsdfIntegrator>(
                        alpha_brain::global_map.config, 
                        alpha_brain::global_map.tsdf_layer.get()
                    );
                    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Integrator locked onto shared map!" RESET);
                }

                // Dummy gray color for voxblox API
                voxblox::Colors empty_colors(batch_cloud.points.size(), voxblox::Color(128, 128, 128));

                // Intergrate batch pointcloud to the map
                this->tsdf_integrator->integratePointCloud(
                    batch_cloud.transfrom, 
                    batch_cloud.points, 
                    empty_colors
                );
            }
            else {
                RCLCPP_WARN(this->theNode->get_logger(), RED "Dropping scan, map memory not ready yet." RESET);
            }
        }
    } // Thread loop back

    #if FLOW
    if(this->running.load(std::memory_order_relaxed)) RCLCPP_ERROR(this->theNode->get_logger(), RED "World Update Thread die unaturally!" RESET);
    #endif    
}

#pragma endregion
