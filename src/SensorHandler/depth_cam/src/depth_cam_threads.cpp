#include "depth_cam/depth_cam_threads.hpp"

#pragma region ProcessingThread class
alpha_brain::ProcessingThread::ProcessingThread(
    const std::string& name,
    rclcpp::Node* thisNode,
    const std::string& topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& hazard_point_queue,
    const std::atomic<bool>& world_update,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue
) : name(name), thisNode(thisNode), topic(topic), tf_buffer(tf_buffer), world_update(world_update), hazard_point_queue(hazard_point_queue), world_update_queue(world_update_queue) {
    // Create subscriber
    this->depth_cam_SUB = thisNode->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->topic,
        rclcpp::SensorDataQoS(),
        std::bind(&alpha_brain::ProcessingThread::DepthCamCallback, this, _1)
    );

    // Init variables
    this->has_tf_body = false;
    this->running.store(true);
    this->hazard_distance_sq.store(Drone::HAZARD_DISTANCE);
    this->done_world_update = false;

    // Spawn thread
    this->processing_thread = std::thread(&ProcessingThread::ConsumerLoop, this);
    RCLCPP_INFO(this->thisNode->get_logger(), GREEN "Spawn worker %s processing thread" RESET, this->name.c_str());
}

alpha_brain::ProcessingThread::~ProcessingThread() {
    this->running.store(false);
    this->msg_queue.enqueue(nullptr);
    if(this->processing_thread.joinable()) this->processing_thread.join();
    RCLCPP_INFO(this->thisNode->get_logger(), BLUE "%s processing thread destructor called" RESET, this->name.c_str());
}

void alpha_brain::ProcessingThread::processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if(!this->has_tf_body) {
        try {
            geometry_msgs::msg::TransformStamped tf_body = this->tf_buffer->lookupTransform(
                this->base_link.get(), // Target frame: body
                msg->header.frame_id, // Current frame: depth camera
                rclcpp::Time(0) // Time stamp don't matter
            );
            this->iso_body = tf2::transformToEigen(tf_body);
            this->has_tf_body = true;
            RCLCPP_INFO(this->thisNode->get_logger(), GREEN "%s depth camera STATIC tf lookup complete" RESET, this->name.c_str());
        }
        catch(const tf2::TransformException& ex) {
            RCLCPP_WARN(this->thisNode->get_logger(), RED "%s point clouds msg denied, cause by STATIC tf: %s" RESET, this->name.c_str(), ex.what());
            return;
        }
    }
    this->msg_queue.enqueue(msg);
}

void alpha_brain::ProcessingThread::DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    processMsg(msg);
}

void alpha_brain::ProcessingThread::updateSafeBubble(const float hazard_distance_sq) {
    this->hazard_distance_sq.store(hazard_distance_sq);
}

void alpha_brain::ProcessingThread::ConsumerLoop() {
    
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the msg
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        this->msg_queue.wait_dequeue(msg);
        while(this->msg_queue.try_dequeue(msg)) {
            RCLCPP_WARN(this->thisNode->get_logger(), YELLOW "%s flushed a mesage" RESET, this->name.c_str());
        } // FLush to only use latest msg
        if(!this->running.load(std::memory_order_relaxed)) break;

        // Load local atomic variables
        bool world_update = this->world_update.load(std::memory_order_relaxed);
        double hazard_distance_sq = this->hazard_distance_sq.load(std::memory_order_relaxed);

        // Check to make sure do world update only once per call
        if(!world_update) this->done_world_update = false;

        // Lookup world frame
        std::optional<Eigen::Isometry3d> iso_world;
        bool has_tf_world = false;
        if(world_update && !this->done_world_update) {
            RCLCPP_INFO(this->thisNode->get_logger(), YELLOW "%s thread called world update" RESET, this->name.c_str());
            try {
                geometry_msgs::msg::TransformStamped tf_world = this->tf_buffer->lookupTransform(
                    "world",
                    msg->header.frame_id, // Current frame: depth camera
                    msg->header.stamp, // Time stamp of the scan
                    rclcpp::Duration::from_nanoseconds(Clock::LOOP_CYCLE_NANOSEC * 2)
                );
                iso_world = tf2::transformToEigen(tf_world);
                has_tf_world = true;
                RCLCPP_INFO(this->thisNode->get_logger(), GREEN "%s depth camera DYNAMIC tf lookup complete" RESET, this->name.c_str());
            }
            catch(const tf2::TransformException& ex) {
                RCLCPP_WARN(this->thisNode->get_logger(), RED "%s transform denied, cause by DYNAMIC tf: %s" RESET, this->name.c_str(), ex.what());
                has_tf_world = false;
            }
        }

        // Prepare intermediate variables
        bool hazard_exist = false;
        std::unique_ptr<octomap::Pointcloud> hazard_cloud;

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

            Eigen::Vector3d raw_point(*itx, *ity, *itz); // Raw from depth camera
            Eigen::Vector3d body_point = this->iso_body * raw_point; // Transformed to body frame
            
            // Body clipping check
            bool x_cliped = (Drone::MIN_X < body_point.x() && body_point.x() < Drone::MAX_X);
            bool y_cliped = (Drone::MIN_Y < body_point.y() && body_point.y() < Drone::MAX_Y);
            bool z_cliped = (Drone::MIN_Z < body_point.z() && body_point.z() < Drone::MAX_Z);
            if(x_cliped && y_cliped && z_cliped) continue;

            // For hazard point
            double distance_sq = body_point.squaredNorm();
            // hazard_distance_sq = 25.0f; // #Test
            if(distance_sq < hazard_distance_sq) {
                if(hazard_exist) {
                    if(hazard_cloud->size() >= HAZARD_BATCH_SIZE) {
                        this->hazard_point_queue.enqueue(std::move(hazard_cloud));
                        hazard_cloud = std::make_unique<octomap::Pointcloud>(); // #CanBeOptimize
                        hazard_cloud->reserve(HAZARD_BATCH_SIZE);
                    }
                }
                else {
                    hazard_cloud = std::make_unique<octomap::Pointcloud>(); // #CanBeOptimize
                    hazard_cloud->reserve(HAZARD_BATCH_SIZE);
                    hazard_exist = true;
                }
                hazard_cloud->push_back(body_point.x(), body_point.y(), body_point.z());
            }

            // For world update
            if(world_update && !this->done_world_update && has_tf_world) {
                Eigen::Vector3d world_point = (*iso_world) * raw_point;
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
    rclcpp::Node* thisNode,
    const int num_worker
) : thisNode(thisNode), num_worker(num_worker), origin(0.0f, 0.0f, 0.0f) {
    // Create Publisher
    this->hazard_voxel_PUB = this->thisNode->create_publisher<alpha_msgs::msg::VoxelBlock>(
        Topic::VOXEL_HAZARD_SEEING,
        rclcpp::SensorDataQoS()
    );

    // Init variables
    this->running.store(true);

    // Spawn persistent thread
    this->hazard_point_thread = std::thread(&HazardPointThread::ConsumerLoop, this);
    RCLCPP_INFO(this->thisNode->get_logger(), GREEN "Spawn Consumer thread Hazard Point" RESET);
}

alpha_brain::HazardPointThread::~HazardPointThread() {
    this->running.store(false);
    this->hazard_point_queue.enqueue(nullptr);
    if(this->hazard_point_thread.joinable()) this->hazard_point_thread.join();
    RCLCPP_INFO(this->thisNode->get_logger(), BLUE "Hazard point thread destructor called" RESET);
}

moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& alpha_brain::HazardPointThread::getQueue() {
    return this->hazard_point_queue;
}

void alpha_brain::HazardPointThread::ConsumerLoop() {
    int worker_finished = 0;
    std::unique_ptr<octomap::OcTree> oc_tree = std::make_unique<octomap::OcTree>(Sensor::OCTREE_VOXEL_RESOLUTION);
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        std::unique_ptr<octomap::Pointcloud> batch_cloud;
        this->hazard_point_queue.wait_dequeue(batch_cloud);
        if(!this->running.load(std::memory_order_relaxed)) break;
        if(!batch_cloud) {
            worker_finished++;
            if(worker_finished >= this->num_worker) {
                oc_tree->updateInnerOccupancy();
                PublishHazardPoint(oc_tree.get());
                oc_tree->clear();
                worker_finished = 0;
            }
            continue;
        }

        // Put the batch cloud to octree
        oc_tree->insertPointCloud((*batch_cloud), origin, Sensor::DEPTH_CAM_RANGE, true);
    }
}

void alpha_brain::HazardPointThread::PublishHazardPoint(const octomap::OcTree *oc_tree) {
    alpha_msgs::msg::Point32Array pa;
    pa.points.reserve(128);
    for(auto it = oc_tree->begin_leafs(), end = oc_tree->end_leafs(); it != end; ++it) {
        if(oc_tree->isNodeOccupied(*it)) {
            octomap::point3d oc_point = it.getCoordinate();
            geometry_msgs::msg::Point32 point;
            point.x = oc_point.x();
            point.y = oc_point.y();
            point.z = oc_point.z();
            pa.points.push_back(point);
        }
    }

    if(pa.points.empty()) {
        RCLCPP_INFO(this->thisNode->get_logger(), YELLOW "Obstacle clear" RESET);
        return;
    }
    alpha_msgs::msg::VoxelBlock msg;
    msg.header.frame_id = this->base_link.get();
    msg.header.stamp = this->thisNode->get_clock()->now();
    msg.size = pa.points.size();
    msg.point_array = pa;
    this->hazard_voxel_PUB->publish(msg);
    RCLCPP_INFO(this->thisNode->get_logger(), GREEN "Published %d hazard points" RESET, pa.points.size());
}

#pragma endregion

#pragma region WorlUpdateThread class

alpha_brain::WorldUpdateThread::WorldUpdateThread(
    rclcpp::Node* thisNode,
    const int num_worker
) : thisNode(thisNode), num_worker(num_worker) {
    // Init variables
    this->running.store(false);

    // Create wall timer
    world_update_TIME = this->thisNode->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
        std::bind(&alpha_brain::WorldUpdateThread::doWorldUpdate, this)
    );
}

alpha_brain::WorldUpdateThread::~WorldUpdateThread() {
    this->running.store(false);
    this->world_update_queue.enqueue(nullptr);
    if(this->world_update_thread.joinable()) this->world_update_thread.join();
    RCLCPP_INFO(this->thisNode->get_logger(), BLUE "World update thread destructor called" RESET);
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
    RCLCPP_INFO(this->thisNode->get_logger(), GREEN "Spawn Consumer thread World Update" RESET);
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
    if(has_data && worker_finished >= 3) RCLCPP_INFO(this->thisNode->get_logger(), GREEN "Wolrd update complete" RESET);
    else if(has_data && worker_finished < 3) RCLCPP_WARN(this->thisNode->get_logger(), YELLOW "Wolrd update incomplete" RESET);
    else RCLCPP_WARN(this->thisNode->get_logger(), PINK "Wolrd update empty" RESET);
    // Thread naturally die here
}

#pragma endregion