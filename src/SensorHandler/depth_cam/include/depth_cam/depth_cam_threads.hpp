#pragma once

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
// #include <octomap/octomap.h>

#include <thread>
#include <atomic>
#include <optional>
#include <Eigen/Dense>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/blockingconcurrentqueue.h"
#include "global_utils/alpha_brain.hpp"
#include "alpha_msgs/msg/vector_field_histogram.hpp"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/layer.h"
#include "voxblox/integrator/tsdf_integrator.h"

#define DEBUG 1
#define TIME_ANALYSE 1

#ifndef DEBUG
    #define DEBUG 0
#endif

#ifndef TIME_ANALYSE
    #define TIME_ANALYSE 0
#endif


namespace alpha_brain {

constexpr int HAZARD_BATCH_SIZE = 128; // #CanBeOptimize
constexpr int WORLD_BATCH_SIZE = 512;

using std::placeholders::_1;

struct VoxbloxBatch {
    voxblox::Pointcloud points;
    voxblox::Transformation transfrom;
};

class ProcessingThread {
public:
    ProcessingThread(
        const std::string& name,
        rclcpp::Node* theNode,
        const std::string& topic,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>>& hazard_point_queue,
        const std::atomic<bool>& world_update,
        moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue,
        time_utils::TimeAnalyzer* analyzer
    );
    ~ProcessingThread();
    void processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateSafeBubble(const float hazard_distance_sq);

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_SUB;

    // Data
    std::string name;
    rclcpp::Node* theNode;
    const std::string& topic;

    Name::Dynamic::BASE_LINK base_link;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    bool has_tf_body;
    Eigen::Isometry3f iso_body;

    std::atomic<float> hazard_distance;

    const std::atomic<bool>& world_update;
    bool done_world_update;
    
    std::atomic<bool> running;
    std::thread processing_thread;

    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr> msg_queue;
    moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>>& hazard_point_queue; // Send in batch of smaller point cloud
    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue; // Send in batch of smaler point cloud
    
    // Methods
    void ConsumerLoop();

    // Callbacks
    void DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // debugs
    time_utils::TimeAnalyzer* analyzer;

};

// Publish hazard point to ReactiveOA Node
class HazardPointThread {
public:
    HazardPointThread(
        rclcpp::Node* theNode,
        const int num_worker
    );
    ~HazardPointThread();

    moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>>& getQueue();

private:
    // Publishers
    rclcpp::Publisher<alpha_msgs::msg::VectorFieldHistogram>::SharedPtr hazard_voxel_PUB;

    // Data
    rclcpp::Node* theNode;

    Name::Dynamic::BASE_LINK base_link;

    const int num_worker;

    std::atomic<bool> running;
    std::thread hazard_point_thread;

    moodycamel::BlockingConcurrentQueue<std::vector<Eigen::Vector3f>> hazard_point_queue;

    // Methods
    void ConsumerLoop();
    void PublishHazardPoint(const std::bitset<Sensor::VFH_TOTAL_BINS>& VFH, const Eigen::Vector3f& sum_repulsive);

};

// Put new scan to voxblox map
class WorldUpdateThread {
public:
    WorldUpdateThread(
        rclcpp::Node* theNode,
        const int num_worker
    );
    ~WorldUpdateThread();

    const std::atomic<bool>& getStatus();
    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& getQueue();
    void doWorldUpdate();
    
private:
    // Timers
    rclcpp::TimerBase::SharedPtr world_update_TIME;

    // Data
    rclcpp::Node* theNode;

    Name::Dynamic::BASE_LINK base_link;

    const int num_worker;

    std::atomic<bool> running;
    std::thread world_update_thread;

    moodycamel::BlockingConcurrentQueue<VoxbloxBatch> world_update_queue;

    // The tool that actually shoots the rays into the map
    std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator = nullptr;

    // Methods
    void ConsumerLoop();
};

} // namespace alpha_brain