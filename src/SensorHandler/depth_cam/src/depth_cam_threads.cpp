#include "depth_cam/depth_cam_threads.hpp"

#pragma region ProcessingThread class

alpha_brain::ProcessingThread::ProcessingThread(
    const std::string& name,
    rclcpp::Node* theNode,
    const std::string& topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue,
    time_utils::TimeAnalyzer* analyzer
) : 
    name(name), 
    theNode(theNode), 
    topic(topic), 
    tf_buffer(tf_buffer), 
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
    this->running.store(true);

    // Spawn thread
    this->processing_thread = std::thread(&ProcessingThread::ProducerLoop, this);

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
    this->msg_queue.enqueue(msg);
}

void alpha_brain::ProcessingThread::DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    processMsg(msg);
}

void alpha_brain::ProcessingThread::ProducerLoop() {
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the msg
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        this->msg_queue.wait_dequeue(msg);
        while(this->msg_queue.try_dequeue(msg)) {
            RCLCPP_WARN_THROTTLE(this->theNode->get_logger(),*this->theNode->get_clock(), 1000, YELLOW "%s flushed a mesage" RESET, this->name.c_str());
        } // FLush to only use latest msg
        if(!this->running.load(std::memory_order_relaxed)) break;

        #if TIME_ANALYSE
        std::string seg_name = "batch ";
        seg_name.append(msg->header.frame_id);
        this->analyzer->start_segment(seg_name);
        #endif

        // Lookup world frame
        std::optional<voxblox::Transformation> voxblox_tf_world;
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
            RCLCPP_WARN_THROTTLE(this->theNode->get_logger(), *this->theNode->get_clock(), 1000, RED "%s transform denied, cause by DYNAMIC tf: %s" RESET, this->name.c_str(), ex.what());
            continue;
        }

        if(!voxblox_tf_world.has_value()) continue;

        // Prepare intermediate variables
        VoxbloxBatch world_update_cloud;
        world_update_cloud.points.reserve(WORLD_BATCH_SIZE);
        world_update_cloud.transfrom = voxblox_tf_world.value();

        sensor_msgs::PointCloud2Iterator<float> itx(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> ity(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> itz(*msg, "z");

        // Iterate through point cloud
        for(; itx != itx.end(); ++itx, ++ity, ++itz) {
            if(!std::isfinite(*itx) || !std::isfinite(*ity) || !std::isfinite(*itz)) continue;

            Eigen::Vector3f raw_point(*itx, *ity, *itz); // Raw from depth camera
            
            //_ For world update
            if(world_update_cloud.points.size() >= WORLD_BATCH_SIZE) {
                this->world_update_queue.enqueue(std::move(world_update_cloud));
                world_update_cloud = VoxbloxBatch(); // #CanBeOptimize
                world_update_cloud.points.reserve(WORLD_BATCH_SIZE);
                world_update_cloud.transfrom = voxblox_tf_world.value();
            }
            world_update_cloud.points.push_back(raw_point.cast<voxblox::FloatingPoint>());
        }

        // Flush the left over batch
        if(world_update_cloud.points.size() > 0) {
            this->world_update_queue.enqueue(std::move(world_update_cloud));
        }

        #if TIME_ANALYSE
        this->analyzer->stop_segment(seg_name);
        #endif
    }
}

#pragma endregion

#pragma region WorlUpdateThread class

alpha_brain::WorldUpdateThread::WorldUpdateThread(
    rclcpp::Node* theNode,
    time_utils::TimeAnalyzer* analyzer
) : 
    theNode(theNode), 
    analyzer(analyzer)
{
    // Init variables
    this->running.store(true);
    
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

moodycamel::BlockingConcurrentQueue<alpha_brain::VoxbloxBatch>& alpha_brain::WorldUpdateThread::getQueue() {
    return this->world_update_queue;
}

void alpha_brain::WorldUpdateThread::ConsumerLoop() {
    // Thread loop
    while(this->running.load(std::memory_order_relaxed)) {
        // Dequeue the batch of point cloud
        VoxbloxBatch batch_cloud;
        this->world_update_queue.wait_dequeue(batch_cloud);

        // Check stop thread condition
        if(!this->running.load(std::memory_order_relaxed)) break;

        // Skip empty flush batches
        if(batch_cloud.points.empty()) continue;

        // Receive valid batch
        #if FLOW
        RCLCPP_INFO(this->theNode->get_logger(), GRAY "Receive batch with %d points" RESET, batch_cloud.points.size());
        #endif

        #if TIME_ANALYSE
        this->analyzer->start_segment("consumer_loop");
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
                    RCLCPP_INFO(this->theNode->get_logger(), GREEN "Integrator locked onto shared map" RESET);
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
                RCLCPP_WARN_SKIPFIRST_THROTTLE(this->theNode->get_logger(),*this->theNode->get_clock(), 1000, RED "Dropping scan, map memory not ready yet." RESET);
            }
        }

        #if TIME_ANALYSE
        this->analyzer->stop_segment("consumer_loop");
        #endif

    } // Thread loop back

    #if FLOW
    if(this->running.load(std::memory_order_relaxed)) RCLCPP_ERROR(this->theNode->get_logger(), RED "World Update Thread die unaturally!" RESET);
    #endif    
}

#pragma endregion