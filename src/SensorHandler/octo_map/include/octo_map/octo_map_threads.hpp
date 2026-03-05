#pragma once

// #include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap/octomap.h>

#include <thread>
// #include <condition_variable>
#include <atomic>
#include <optional>
#include <Eigen/Dense>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/blockingconcurrentqueue.h" 

constexpr int MAX_BATCH_SIZE = 1024;

class ProcessingThread {
public:
    ProcessingThread(
        const std::string& name,
        const std::string& inputTopic,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::atomic<bool>& running,
        std::atomic<bool>& world_update,
        moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr>& msg_queue,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& hazard_point_queue,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue
    );
    ~ProcessingThread();
    void processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    rclcpp::Node* m_theNode;

    std::string m_name;
    std::string m_inputTopic;

    bool m_has_tf_body;
    bool m_has_tf_world;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    Eigen::Isometry3d m_iso_body;

    float m_safe_distance_sq_PLACEHOLDER = 9; // metter square

    std::atomic<bool>& m_running;
    std::atomic<bool>& m_world_update;

    std::thread m_processing_thread;

    std::atomic<uint8_t> m_msg_queue_size;
    std::atomic<bool> m_msg_queue_flush;

    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr>& m_msg_queue;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_hazard_point_queue; // Send in batch of smaler point cloud
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_world_update_queue; // Send in batch of smaler point cloud
    
    void WorkerLoop();

};

class WorldUpdateThread {
public:
    WorldUpdateThread(
        std::atomic<bool>& running,
        std::atomic<bool>& update_world,
        moodycamel::BlockingConcurrentQueue<octomap::Pointcloud>& world_update_queue
    );
    ~WorldUpdateThread();
    void ConsumerLoop();

private:
    std::atomic<bool>& running;
    std::atomic<bool>& update_world;
    std::thread world_update_thread;
    moodycamel::BlockingConcurrentQueue<octomap::Pointcloud>& world_update_queue;
};

