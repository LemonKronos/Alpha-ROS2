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
#include "alpha_msgs/msg/vector_field_histogram.hpp"

namespace alpha_brain {

// class DepthCamNode; // Forward declaration

constexpr int HAZARD_BATCH_SIZE = 128; // #CanBeOptimize
constexpr int WORLD_BATCH_SIZE = 512;

using std::placeholders::_1;

class ProcessingThread {
public:
    ProcessingThread(
        const std::string& name,
        rclcpp::Node* theNode,
        const std::string& topic,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>>& hazard_point_queue,
        const std::atomic<bool>& world_update,
        moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue
    );
    ~ProcessingThread();
    void processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateSafeBubble(const float hazard_distance_sq);

private:
    std::string name;
    rclcpp::Node* theNode;
    const std::string& topic;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_SUB;

    Name::Dynamic::BASE_LINK base_link;
    bool has_tf_body;
    Eigen::Isometry3f iso_body;

    std::atomic<float> hazard_distance;

    std::atomic<bool> running;
    const std::atomic<bool>& world_update;
    bool done_world_update;

    std::thread processing_thread;

    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr> msg_queue;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>>& hazard_point_queue; // Send in batch of smaller point cloud
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& world_update_queue; // Send in batch of smaler point cloud
    
    void ConsumerLoop();

    void DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};

// Publish hazard point to ReactiveOA Node
class HazardPointThread {
public:
    HazardPointThread(
        rclcpp::Node* theNode,
        const int num_worker
    );
    ~HazardPointThread();
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>>& getQueue();

private:
    rclcpp::Node* theNode;

    Name::Dynamic::BASE_LINK base_link;
    rclcpp::Publisher<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr hazard_voxel_PUB;

    const int num_worker;
    const octomap::point3d origin; 
    std::atomic<bool> running;
    std::thread hazard_point_thread;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<std::vector<Eigen::Vector3f>>> hazard_point_queue;

    void ConsumerLoop();
    void PublishHazardPoint(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& closest_point);

};

// Not really sure what it do right now
class WorldUpdateThread {
public:
    WorldUpdateThread(
        rclcpp::Node* theNode,
        const int num_worker
    );
    ~WorldUpdateThread();
    const std::atomic<bool>& getStatus();
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>>& getQueue();
    void doWorldUpdate();
    
private:
    rclcpp::Node* theNode;

    Name::Dynamic::BASE_LINK base_link;

    rclcpp::TimerBase::SharedPtr world_update_TIME;

    const int num_worker;

    std::atomic<bool> running;
    std::thread world_update_thread;
    moodycamel::BlockingConcurrentQueue<std::unique_ptr<octomap::Pointcloud>> world_update_queue;

    void ConsumerLoop();
};

} // namespace alpha_brain