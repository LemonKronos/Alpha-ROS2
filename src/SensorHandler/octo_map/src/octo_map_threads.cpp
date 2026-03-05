#include "octo_map/octo_map_threads.hpp"

ProcessingThread::ProcessingThread(
    const std::string& m_name,
    const std::string& m_inputTopic,
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer,
    std::atomic<bool>& m_running,
    std::atomic<bool>& m_world_update,
    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr>& m_msg_queue,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_hazard_point_queue,
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_world_update_queue
) : m_name(m_name), m_inputTopic(m_inputTopic), m_tf_buffer(m_tf_buffer), m_running(m_running), m_world_update(m_world_update), m_msg_queue(m_msg_queue), m_hazard_point_queue(m_hazard_point_queue), m_world_update_queue(m_world_update_queue) {
    m_processing_thread = std::thread(&ProcessingThread::WorkerLoop, this, NULL);

    // Init variables
    m_has_tf_body = false;
    m_has_tf_world = false;
    m_msg_queue_size.store(0);
    m_msg_queue_flush.store(false);
}

ProcessingThread::~ProcessingThread() {
    m_msg_queue.enqueue(nullptr);
    if(m_processing_thread.joinable()) m_processing_thread.join();
}

void ProcessingThread::processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if(m_has_tf_body) {
        m_msg_queue.enqueue(msg);
        m_msg_queue_size++;
        if(m_msg_queue_size.load() >= 3) m_msg_queue_flush.store(true);
    }
    else {
        try {
            geometry_msgs::msg::TransformStamped tf_body = m_tf_buffer->lookupTransform(
                "alpha_minus_2_0/base_link", // Target frame: body
                msg->header.frame_id, // Current frame: depth camera
                rclcpp::Time(0) // Time stamp don't matter
            );
            m_iso_body = tf2::transformToEigen(tf_body);
            m_has_tf_body = true;
            m_msg_queue.enqueue(msg);
            m_msg_queue_size++;
        }
        catch(const tf2::TransformException& ex) {
            RCLCPP_WARN(m_theNode->get_logger(), RED "%s point clouds msg denied, cause by tf: %s" RESET, this->m_name, ex.what());
        }
    }
}

void ProcessingThread::WorkerLoop() {
    
    while(m_running.load()) {
        // Dequeue the msg
        sensor_msgs::msg::PointCloud2::SharedPtr msg;
        m_msg_queue.wait_dequeue(msg);
        if(!m_running.load()) break;

        // Lookup world frame
        std::optional<Eigen::Isometry3d> iso_world;
        if(m_world_update.load()) {
            try {
                geometry_msgs::msg::TransformStamped tf_world = m_tf_buffer->lookupTransform(
                    "world", // Are we sure it world?
                    msg->header.frame_id, // Current frame: depth camera
                    msg->header.stamp, // Time stamp of the scan
                    rclcpp::Duration::from_nanoseconds(Clock::LOOP_CYCLE_NANOSEC)
                );
                iso_world = tf2::transformToEigen(tf_world);
                m_has_tf_world = true;
            }
            catch(const tf2::TransformException& ex) {
                RCLCPP_WARN(m_theNode->get_logger(), RED "%s transform denied, cause by tf: %s" RESET, this->m_name, ex.what());
                m_has_tf_world = false;
            }
        }

        // Prepare intermediate variables
        bool hazard_exist = false;
        std::unique_ptr<octomap::Pointcloud> hazard_cloud;

        std::unique_ptr<octomap::Pointcloud> world_update_cloud;
        if(m_world_update.load()) world_update_cloud = std::make_unique<octomap::Pointcloud>(MAX_BATCH_SIZE);

        sensor_msgs::PointCloud2Iterator<float> itx(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> ity(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> itz(*msg, "z");

        // Iterate through point cloud
        for(; itx != itx.end(); ++itx, ++ity, ++itz) {
            if(std::isnan(*itx) || std::isnan(*ity) || std::isnan(*itz)) continue;

            Eigen::Vector3f raw_point(*itx, *ity, *itz);
            
            // For hazard point
            Eigen::Vector3d body_point = m_iso_body * raw_point;
            int distance_sq = body_point.squaredNorm();
            if(distance_sq < m_safe_distance_sq_PLACEHOLDER) {
                if(hazard_exist) {
                    if(hazard_cloud->size() >= MAX_BATCH_SIZE) {
                        m_hazard_point_queue.enqueue(std::move(hazard_cloud));
                        hazard_cloud = std::make_unique<octomap::Pointcloud>(MAX_BATCH_SIZE);
                    }
                }
                else {
                    hazard_cloud = std::make_unique<octomap::Pointcloud>(MAX_BATCH_SIZE);
                    hazard_exist = true;
                }
                hazard_cloud->push_back(body_point.x(), body_point.y(), body_point.z());
            }

            // For world update
            if(m_world_update.load()) {
                Eigen::Vector3d world_point = (*iso_world) * raw_point;
                if(world_update_cloud->size() >= MAX_BATCH_SIZE) {
                    m_world_update_queue.enqueue(std::move(world_update_cloud));
                    world_update_cloud = std::make_unique<octomap::Pointcloud>(MAX_BATCH_SIZE);
                }
                world_update_cloud->push_back(world_point.x(), world_point.y(), world_point.z());
            }
        }

        // Flush the left over batch
        if(hazard_cloud->size() > 0) m_hazard_point_queue.enqueue(std::move(hazard_cloud));
        if(world_update_cloud->size() > 0) m_world_update_queue.enqueue(std::move(world_update_cloud));

        // Send empty batch to indicate end of msg
        m_hazard_point_queue.enqueue(nullptr);
        m_world_update_queue.enqueue(nullptr);
    }
}
