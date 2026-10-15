#pragma once

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <thread>
#include <atomic>
#include <optional>
#include <Eigen/Dense>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/blockingconcurrentqueue.h"
#include "global_utils/alpha_brain.hpp"
#include "voxblox/core/common.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/layer.h"
#include "voxblox/integrator/tsdf_integrator.h"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

// Local define
#define DEBUG (ALLOW_DEBUG & 1)
#define FLOW (ALLOW_DEBUG & 0)
#define VISUALIZE (ALLOW_DEBUG & 0)
#define TIME_ANALYSE (ALLOW_DEBUG & 0)

namespace alpha_brain {

constexpr int WORLD_BATCH_SIZE = 4800;

using std::placeholders::_1;

struct VoxbloxBatch {
    voxblox::Pointcloud points;
    voxblox::Transformation transfrom;
};

// Handle the depth camera sensors
class ProcessingThread {
public:
    ProcessingThread(
        const std::string& name,
        rclcpp::Node* theNode,
        const std::string& topic,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue,
        time_utils::TimeAnalyzer* analyzer
    );
    ~ProcessingThread();
    void processMsg(sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_SUB;

    // Data
    std::string name;
    rclcpp::Node* theNode;
    const std::string& topic;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    
    std::atomic<bool> running;
    std::thread processing_thread;

    moodycamel::BlockingConcurrentQueue<sensor_msgs::msg::PointCloud2::SharedPtr> msg_queue;
    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& world_update_queue; // Send in batch of smaler point cloud
    
    // Methods
    void ProducerLoop();

    // Callbacks
    void DepthCamCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // debugs
    time_utils::TimeAnalyzer* analyzer;

};

// Put new scan to voxblox map
class WorldUpdateThread {
public:
    WorldUpdateThread(
        rclcpp::Node* theNode,
        time_utils::TimeAnalyzer* analyzer
    );
    ~WorldUpdateThread();

    moodycamel::BlockingConcurrentQueue<VoxbloxBatch>& getQueue();
    
private:
    // Data
    rclcpp::Node* theNode;

    std::atomic<bool> running;
    std::thread world_update_thread;

    moodycamel::BlockingConcurrentQueue<VoxbloxBatch> world_update_queue;

    // The tool that actually shoots the rays into the map
    std::unique_ptr<voxblox::FastTsdfIntegrator> tsdf_integrator = nullptr;

    // Methods
    void ConsumerLoop();

    // debugs
    time_utils::TimeAnalyzer* analyzer;
};

} // namespace alpha_brain