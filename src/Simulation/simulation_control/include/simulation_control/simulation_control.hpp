#ifndef SIMULATION_CONTROL_HPP_
#define SIMULATION_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp>

#include "alpha_msgs/msg/record_control.hpp"
#include "global_utils/system_config.hpp"

class SimulationControlNode : public rclcpp::Node {
public:
    SimulationControlNode();
    ~SimulationControlNode() = default; // No need to destroy CV windows anymore

private:
    // --- Config ---
    bool is_paused_;

    // --- ROS Interfaces ---
    rclcpp::Subscription<alpha_msgs::msg::RecordControl>::SharedPtr sub_record_control_;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr client_gz_;

    // --- Methods ---
    void record_control_callback(const alpha_msgs::msg::RecordControl::SharedPtr msg);
    
    // World Control Helper
    void send_pause_command(bool pause);
};

#endif // SIMULATION_CONTROL_HPP_