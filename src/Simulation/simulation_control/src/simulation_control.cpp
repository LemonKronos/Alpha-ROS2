#include "simulation_control/simulation_control.hpp"

using namespace std::chrono_literals;

SimulationControlNode::SimulationControlNode() 
    : Node("simulation_control"), 
      is_paused_(false)
{
    // 1. Record Control Subscription (Reliable)
    sub_record_control_ = this->create_subscription<alpha_msgs::msg::RecordControl>(
        Topic::LOGGER_RECORD, 
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
        std::bind(&SimulationControlNode::record_control_callback, this, std::placeholders::_1));

    // 2. World Control Client
    client_gz_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>(Service::WORLD_CONTROL);

    RCLCPP_INFO(this->get_logger(), GREEN "Simulation Control Node Started. Listening to %s for Pause." RESET, Topic::LOGGER_RECORD);
}

void SimulationControlNode::record_control_callback(const alpha_msgs::msg::RecordControl::SharedPtr msg) {
    // Only act if the state actually changes
    if (msg->pause && !is_paused_) {
        // Command says PAUSE, we are running -> PAUSE
        is_paused_ = true;
        send_pause_command(true);
    } 
    else if (!msg->pause && is_paused_) {
        // Command says RESUME, we are paused -> RESUME
        is_paused_ = false;
        send_pause_command(false);
    }
}

void SimulationControlNode::send_pause_command(bool pause) {
    if (!client_gz_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "World control service not ready!");
        return;
    }

    auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    req->world_control.pause = pause;

    client_gz_->async_send_request(req);

    if (pause) {
        RCLCPP_INFO(this->get_logger(), "Command: PAUSE Simulation");
    } else {
        RCLCPP_INFO(this->get_logger(), "Command: RESUME Simulation");
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulationControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}