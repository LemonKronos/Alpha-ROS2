#pragma once

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap/octomap.h>

#include <thread>
#include <atomic>
#include <optional>
#include <Eigen/Dense>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/blockingconcurrentqueue.h"
#include "ros2_msgs/msg/voxel_block.hpp"

namespace alpha_brain {

class DepthCamNode; // Forward declaration

constexpr int HAZARD_BATCH_SIZE = 256; // #CanBeOptimize
constexpr int WORLD_BATCH_SIZE = 1024;

using std::placeholders::_1;

class ProcessingThread {
public:
    ProcessingThread(
        const std::string& name,
        rclcpp::Node* thisNode,
        const std::string& topic,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& hazard_point_queue,
        const std::atomic<bool>& world_update,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue
    );
    ~ProcessingThread();
    void processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateSafeBubble(const float hazard_distance_sq);

private:
    std::string m_name;

    rclcpp::Node* m_thisNode;

    const std::string& m_topic;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_depth_cam_SUB;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;

    bool m_has_tf_body;
    Eigen::Isometry3d m_iso_body;

    std::atomic<float> m_hazard_distance_sq; // metter square

    std::atomic<bool> m_running;
    const std::atomic<bool>& m_world_update;
    bool m_done_world_update;

    std::thread m_processing_thread;

    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr> m_msg_queue;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_hazard_point_queue; // Send in batch of smaler point cloud
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& m_world_update_queue; // Send in batch of smaler point cloud
    
    void ConsumerLoop();

    void DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};

// Publish hazard point to ReactiveOA Node
class HazardPointThread {
public:
    HazardPointThread(
        rclcpp::Node* thisNode,
        const int num_worker
    );
    ~HazardPointThread();
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& getQueue();

private:
    rclcpp::Node* m_thisNode;
    rclcpp::Publisher<ros2_msgs::msg::VoxelBlock>::SharedPtr m_hazard_voxel_PUB;

    const int m_num_worker;
    const octomap::point3d origin; 
    std::atomic<bool> m_running;
    std::thread m_hazard_point_thread;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>> m_hazard_point_queue;

    void ConsumerLoop();
    void PublishHazardPoint(const octomap::OcTree *oc_tree);

};

// Not really sure what it do right now
class WorldUpdateThread {
public:
    WorldUpdateThread(
        rclcpp::Node* thisNode,
        const int num_worker
    );
    ~WorldUpdateThread();
    const std::atomic<bool>& getStatus();
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& getQueue();
    void doWorldUpdate();
    
private:
    rclcpp::Node* m_thisNode;

    rclcpp::TimerBase::SharedPtr world_update_TIME;

    const int m_num_worker;

    std::atomic<bool> m_running;
    std::thread m_world_update_thread;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>> m_world_update_queue;

    void ConsumerLoop();
};

} // namespace alpha_brain