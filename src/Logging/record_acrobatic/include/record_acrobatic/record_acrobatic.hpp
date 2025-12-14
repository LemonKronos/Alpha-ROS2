/**
ROS2 node to record and ouput dataset for AcrobaticOA

episode structure:
    episode_0001/
    ├─ state.npy
    ├─ rgb_front.mp4
    ├─ depth_front.mp4
    ├─ action.npy
    ├─ noob_01.npy
    ├─ overview.mp4
    └─ meta.json

 - state.npy: sensor data
 - rgb_front.mp4: RGB camera front, 640x480 pixels
 - depth_front.mp4: Depth camera front, 640x480 pixels, min range 0.2, max range 30.0 meter
 - action.npy: expert control
 - noob_xx.npy: noob control - also a state data, will later be concatenate to state.npy, can have multiple noob control for a expert action
 - overview.mp4: video record of the whole episode, use as a reference for adding nood control and a overview of the movement
 - meta.json: include meta data for the episode:
    - expert_manuever: name for such expert action
    - fps: system rate
    - timestamp_start: timestamp start episode
    - timestamp_end: timestamp end episode
    - duration: length of record
    - frames: frame count
    - state_dim: state without noob control dimension
    - action_dim: action dimension
 */

#ifndef RECORD_ACROBATIC_NODE_HPP_
#define RECORD_ACROBATIC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <mutex>
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"

// ROS Messages
#include "ros2_msgs/msg/control_interface.hpp"
#include "ros2_msgs/msg/record_control.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "sensor_msgs/msg/image.hpp"

// OpenCV & CV Bridge
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

// JSON & CNPY
#include <nlohmann/json.hpp>
#include <cnpy.h>

using json = nlohmann::json;
namespace fs = std::filesystem;

// Constants
constexpr int TIMEOUT_CYCLES = 3;

struct DataCache {
    double timestamp = 0.0;
    bool valid = false;
};

struct PerceptionCache : DataCache {
    ros2_msgs::msg::FusePerception::SharedPtr msg;
};

struct LidarCache : DataCache {
    ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg;
};

class RecordAcrobaticNode : public rclcpp::Node {
public:
    RecordAcrobaticNode();
    ~RecordAcrobaticNode();

private:
    // --- State ---
    bool recording_;
    fs::path episode_dir_;
    double start_timestamp_;
    
    // Data Buffers (Flat vectors for cnpy)
    std::vector<float> state_buffer_;  // Flattened N x 40
    std::vector<float> action_buffer_; // Flattened N x 7
    size_t frame_count_;

    // Current Action Buffer
    std::vector<float> current_action_;

    // --- Video Writers ---
    cv::VideoWriter vw_rgb_;
    cv::VideoWriter vw_depth_;
    cv::VideoWriter vw_overview_;
    cv::Size dim_input_;
    cv::Size dim_overview_;

    // --- Caches ---
    PerceptionCache cache_perc_;
    LidarCache cache_lidar_close_;
    LidarCache cache_lidar_far_;
    
    // Image caches (CvImages)
    cv_bridge::CvImagePtr cache_img_rgb_;
    cv_bridge::CvImagePtr cache_img_depth_;
    cv_bridge::CvImagePtr cache_img_overview_;

    // --- ROS Interfaces ---
    rclcpp::Subscription<ros2_msgs::msg::RecordControl>::SharedPtr sub_record_control_;
    rclcpp::Subscription<ros2_msgs::msg::ControlInterface>::SharedPtr sub_expert_action_;
    
    rclcpp::Subscription<ros2_msgs::msg::FusePerception>::SharedPtr sub_perception_;
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr sub_lidar_close_;
    rclcpp::Subscription<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr sub_lidar_far_;

    // Dynamic Image Subs
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_depth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_overview_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Methods ---
    void node_loop_callback();
    
    // Helpers
    bool is_alive(const DataCache& cache, double now);
    std::vector<float> extract_sectors(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr& msg);
    std::vector<float> msg_to_action_list(const ros2_msgs::msg::ControlInterface::SharedPtr& msg);
    
    // Dynamic Subs
    void start_image_subs();
    void stop_image_subs();
    
    // Episode Mgmt
    void start_episode();
    void finish_episode();
    
    // Callbacks
    void record_control_callback(const ros2_msgs::msg::RecordControl::SharedPtr msg);
    void expert_action_callback(const ros2_msgs::msg::ControlInterface::SharedPtr msg);
    void perception_callback(const ros2_msgs::msg::FusePerception::SharedPtr msg);
    void lidar_close_callback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    void lidar_far_callback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    
    void img_rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void img_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void img_overview_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // RECORD_ACROBATIC_NODE_HPP_