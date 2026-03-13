#ifndef ADD_NOOB_CONTROL_NODE_HPP_
#define ADD_NOOB_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"

// ROS Messages
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/record_control.hpp"

// External Libs
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <cnpy.h>

namespace fs = std::filesystem;
using json = nlohmann::json;

class AddNoobControlNode : public rclcpp::Node {
public:
    explicit AddNoobControlNode(int ep_num);
    ~AddNoobControlNode();

private:
    // --- Callbacks ---
    void input_callback(const alpha_msgs::msg::ControlInterface::SharedPtr msg);
    void record_control_callback(const alpha_msgs::msg::RecordControl::SharedPtr msg);
    void game_loop();

    // --- Logic Helpers ---
    void start_recording();
    void finish_recording();
    void discard_recording();

    // --- State Variables ---
    fs::path episode_path_;
    size_t total_frames_;
    int fps_;
    
    size_t frame_idx_;
    std::string state_; // WAITING, RECORDING, DISCARDING, PAUSED, SAVING

    // --- Data Buffers ---
    std::vector<float> current_input_; // Current joystick state [7]
    std::vector<float> noob_buffer_;   // Accumulated history (Flattened N x 7)

    // --- Media ---
    cv::VideoCapture cap_;
    cv::Mat last_frame_; // For pause display

    // --- ROS ---
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr sub_input_;
    rclcpp::Subscription<alpha_msgs::msg::RecordControl>::SharedPtr sub_record_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ADD_NOOB_CONTROL_NODE_HPP_