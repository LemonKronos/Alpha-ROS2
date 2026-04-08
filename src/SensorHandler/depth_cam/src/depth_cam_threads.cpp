#include "depth_cam/depth_cam_threads.hpp"

#pragma region ProcessingThread class

alpha_brain::ProcessingThread::ProcessingThread(
    const std::string& name,
    rclcpp::Node* theNode,
    const std::string& topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>>& hazard_point_queue,
    const std::atomic<bool>& world_update,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue
) : name(name), theNode(theNode), topic(topic), tf_buffer(tf_buffer), world_update(world_update), hazard_point_queue(hazard_point_queue), world_update_queue(world_update_queue) {
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
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn worker %s processing thread" RESET, this->name.c_str());
}

alpha_brain::ProcessingThread::~ProcessingThread() {
    this->running.store(false);
    this->msg_queue.enqueue(nullptr);
    if(this->processing_thread.joinable()) this->processing_thread.join();
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "%s processing thread destructor called" RESET, this->name.c_str());
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
        std::optional<Eigen::Isometry3f> iso_world;
        bool has_tf_world = false;
        if(world_update && !this->done_world_update) {
            RCLCPP_INFO(this->theNode->get_logger(), YELLOW "%s thread called world update" RESET, this->name.c_str());
            try {
                geometry_msgs::msg::TransformStamped tf_world = this->tf_buffer->lookupTransform(
                    "world",
                    msg->header.frame_id, // Current frame: depth camera
                    msg->header.stamp, // Time stamp of the scan
                    rclcpp::Duration::from_nanoseconds(Clock::LOOP_CYCLE_NANOSEC * 2)
                );
                iso_world = tf2::transformToEigen(tf_world).cast<float>();
                has_tf_world = true;
                RCLCPP_INFO(this->theNode->get_logger(), GREEN "%s depth camera DYNAMIC tf lookup complete" RESET, this->name.c_str());
            }
            catch(const tf2::TransformException& ex) {
                RCLCPP_WARN(this->theNode->get_logger(), RED "%s transform denied, cause by DYNAMIC tf: %s" RESET, this->name.c_str(), ex.what());
                has_tf_world = false;
            }
        }

        // Prepare intermediate variables>
        bool hazard_exist = false;
        std::unique_ptr<std::vector<Eigen::Vector3f>> hazard_cloud;

        std::unique_ptr<octomap::Pointcloud> world_update_cloud;
        if(world_update && !this->done_world_update) {
            world_update_cloud = std::make_unique<octomap::Pointcloud>(); // #CanBeOptimize
            world_update_cloud->reserve(WORLD_BATCH_SIZE);
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

            // For hazard point
            // hazard_distance_sq = 400.0f; // #Test
            if(body_point.squaredNorm() <= hazard_distance_sq) {
                // Convert to spherical coordinate
                Eigen::Vector3f spherical_body_point = math_utils::toSpherical(body_point);

                // Push into batch
                if(hazard_exist) {
                    if(hazard_cloud->size() >= HAZARD_BATCH_SIZE) {
                        this->hazard_point_queue.enqueue(std::move(hazard_cloud));
                        hazard_cloud = std::make_unique<std::vector<Eigen::Vector3f>>(); // #CanBeOptimize
                        hazard_cloud->reserve(HAZARD_BATCH_SIZE);
                    }
                }
                else {
                    hazard_cloud = std::make_unique<std::vector<Eigen::Vector3f>>(); // #CanBeOptimize
                    hazard_cloud->reserve(HAZARD_BATCH_SIZE);
                    hazard_exist = true;
                }
                hazard_cloud->push_back(spherical_body_point);
            }

            // For world update
            if(world_update && !this->done_world_update && has_tf_world) {
                Eigen::Vector3f world_point = (*iso_world) * raw_point;
                if(world_update_cloud->size() >= WORLD_BATCH_SIZE) {
                    this->world_update_queue.enqueue(std::move(world_update_cloud));
                    world_update_cloud = std::make_unique<octomap::Pointcloud>(); // #CanBeOptimize
                    world_update_cloud->reserve(WORLD_BATCH_SIZE);
                }
                world_update_cloud->push_back(world_point.x(), world_point.y(), world_point.z());
            }
        }

        // Flush the left over batch and Send empty batch to indicate end of msg
        if(hazard_cloud != nullptr && hazard_cloud->size() > 0) this->hazard_point_queue.enqueue(std::move(hazard_cloud));
        this->hazard_point_queue.enqueue(nullptr);
        if(world_update && !this->done_world_update) {
            if(world_update_cloud != nullptr && world_update_cloud->size() > 0) this->world_update_queue.enqueue(std::move(world_update_cloud));   
            this->world_update_queue.enqueue(nullptr);
            this->done_world_update = true;
        }
    }
}

#pragma endregion

#pragma region HazardPointThread class

alpha_brain::HazardPointThread::HazardPointThread(
    rclcpp::Node* theNode,
    const int num_worker
) : theNode(theNode), num_worker(num_worker), origin(0.0f, 0.0f, 0.0f) {
    // Create Publisher
    this->hazard_voxel_PUB = this->theNode->create_publisher<alpha_msgs::msg::VectorFieldHistogram>(
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
            + ", but got"
            + std::to_string(test_msg.vfh_part.size());
        RCLCPP_FATAL(this->theNode->get_logger(), RED "%s" RESET, error_msg.c_str());
        throw std::runtime_error(error_msg);
    }

    // Spawn persistent thread
    this->hazard_point_thread = std::thread(&HazardPointThread::ConsumerLoop, this);
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn Consumer thread Hazard Point" RESET);
}

alpha_brain::HazardPointThread::~HazardPointThread() {
    this->running.store(false);
    this->hazard_point_queue.enqueue(nullptr);
    if(this->hazard_point_thread.joinable()) this->hazard_point_thread.join();
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "Hazard point thread destructor called" RESET);
}

moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>>& alpha_brain::HazardPointThread::getQueue() {
    return this->hazard_point_queue;
}

void alpha_brain::HazardPointThread::ConsumerLoop() {
    int worker_finished = 0;
    std::bitset<Sensor::VFH_TOTAL_BINS> VFH;
    VFH.reset();
    Eigen::Vector3f repulsive_direction;
    repulsive_direction.setZero();
    float repulsive_value = FLT_MAX;

    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        std::unique_ptr<std::vector<Eigen::Vector3f>> batch_cloud;
        this->hazard_point_queue.wait_dequeue(batch_cloud);

        if(!this->running.load(std::memory_order_relaxed)) break;

        if(batch_cloud == nullptr) {
            worker_finished++;
            if(worker_finished >= this->num_worker) {
                // Compute repulsive vector
                // Could do sum of S_vector if VFH[index] != 1 to get roughly this
                for(int index = 0; index < Sensor::VFH_TOTAL_BINS; index++) {
                    if(VFH[index] == 0) continue;

                    int row = index / Sensor::VFH_AZIMUTH_BINS;
                    int col = index % Sensor::VFH_AZIMUTH_BINS;

                    float yaw = (col * Sensor::VFH_RESOLUTION) - M_PI + Sensor::VFH_RESOLUTION / 2.0f;
                    float pitch = (row * Sensor::VFH_RESOLUTION) - M_PI_2 + Sensor::VFH_RESOLUTION / 2.0f;
                    Eigen::Vector3f bin_direction = math_utils::toCartesian({yaw, pitch, 1.0f});

                    repulsive_direction += bin_direction;
                }

                repulsive_direction.normalize();
                repulsive_direction *= repulsive_value;

                // Send to publish
                PublishHazardPoint(VFH, repulsive_direction);
                VFH.reset();
                repulsive_direction.setZero();
                repulsive_value = FLT_MAX;
                worker_finished = 0;
            }
            continue;
        }

        // Put the batch cloud to VFH #CanBeOptimize maybe try do the repulsive update in here
        for(const auto &point : *batch_cloud) {
            // Get scaling
            float scale_distance = std::min(1.0f, Drone::HAZARD_DISTANCE / point.z());
            float scale_angle = std::asin(scale_distance);
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
            repulsive_value = std::min(repulsive_value, point.z());
        }
    }
}

void alpha_brain::HazardPointThread::PublishHazardPoint(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& sum_repulsive) {
    alpha_msgs::msg::VectorFieldHistogram msg;

    // Check if clear
    if(VFH.none()) {
        RCLCPP_INFO(this->theNode->get_logger(), YELLOW "Obstacle clear" RESET);
        return;
    }

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
    this->hazard_voxel_PUB->publish(msg);
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Hazard VFH show occupied %d cells" RESET, VFH.count());
}

#pragma endregion

#pragma region WorlUpdateThread class

alpha_brain::WorldUpdateThread::WorldUpdateThread(
    rclcpp::Node* theNode,
    const int num_worker
) : theNode(theNode), num_worker(num_worker) {
    // Init variables
    this->running.store(false);

    // Create wall timer
    world_update_TIME = this->theNode->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
        std::bind(&alpha_brain::WorldUpdateThread::doWorldUpdate, this)
    );
}

alpha_brain::WorldUpdateThread::~WorldUpdateThread() {
    this->running.store(false);
    this->world_update_queue.enqueue(nullptr);
    if(this->world_update_thread.joinable()) this->world_update_thread.join();
    RCLCPP_INFO(this->theNode->get_logger(), BLUE "World update thread destructor called" RESET);
}

const std::atomic<bool>& alpha_brain::WorldUpdateThread::getStatus() {
    return this->running;
}

moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& alpha_brain::WorldUpdateThread::getQueue() {
    return this->world_update_queue;
}

void alpha_brain::WorldUpdateThread::doWorldUpdate() {
    this->running.store(false);
    this->world_update_queue.enqueue(nullptr);
    if (this->world_update_thread.joinable()) this->world_update_thread.join();
    std::unique_ptr<octomap::Pointcloud> flush_batch;
    while(this->world_update_queue.try_dequeue(flush_batch)){};

    this->running.store(true);
    this->world_update_thread = std::thread(&WorldUpdateThread::ConsumerLoop, this);
    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Spawn Consumer thread World Update" RESET);
}

void alpha_brain::WorldUpdateThread::ConsumerLoop() {
    uint8_t worker_finished = 0;
    bool has_data = false;
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        std::unique_ptr<octomap::Pointcloud> batch_cloud;
        this->world_update_queue.wait_dequeue(batch_cloud);
        if(!this->running.load(std::memory_order_relaxed)) break;
        if(!batch_cloud) {
            worker_finished++;
            if(worker_finished >= this->num_worker) break;
            else continue;
        }

        has_data = true;

        // I had no idea whether this batch cloud will get push to the existing octomap or make new octomap, so for now the batch cloud just sit here and die
    }
    this->running.store(false);
    if(has_data && worker_finished >= 3) RCLCPP_INFO(this->theNode->get_logger(), GREEN "Wolrd update complete" RESET);
    else if(has_data && worker_finished < 3) RCLCPP_WARN(this->theNode->get_logger(), YELLOW "Wolrd update incomplete" RESET);
    else RCLCPP_INFO(this->theNode->get_logger(), PINK "Wolrd update empty" RESET);
    // Thread naturally die here
}

#pragma endregion
