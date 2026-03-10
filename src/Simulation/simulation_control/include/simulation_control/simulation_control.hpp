#ifndef SIMULATION_CONTROL_HPP_
#define SIMULATION_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <mutex>

#include "ros2_msgs/msg/record_control.hpp"
#include "global_utils/system_config.hpp"

constexpr const char* WINDOW_OVERVIEW_FPV = "Alpha FPV";

class SimulationControlNode : public rclcpp::Node {
public:
    SimulationControlNode();
    ~SimulationControlNode();

private:
    // --- Config ---
    bool is_paused_;

    // --- ROS Interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
    rclcpp::Subscription<ros2_msgs::msg::RecordControl>::SharedPtr sub_record_control_; // NEW
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr client_gz_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Data Management ---
    std::mutex mtx_;
    cv::Mat current_frame_;
    bool new_frame_available_;

    // --- Methods ---
    void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void record_control_callback(const ros2_msgs::msg::RecordControl::SharedPtr msg); // NEW

    // Main Node Loop (Visuals + Service Logic)
    void node_loop();
    
    // World Control Helper
    void send_pause_command(bool pause);
};

#endif // SIMULATION_CONTROL_HPP_