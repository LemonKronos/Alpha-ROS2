#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "ros2_msgs/msg/control_interface.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
// #include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include <algorithm>

constexpr float ATTITUDE_THRESHOLD = 0.25f;

using std::placeholders::_1;

class FinalizeControlNode : public rclcpp::Node {
public:
    FinalizeControlNode();
    ~FinalizeControlNode();

private:
    // Publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_PUB;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_PUB;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_set_PUB;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_set_PUB;

    // Subcriber
    rclcpp::Subscription<ros2_msgs::msg::ControlInterface>::SharedPtr final_ctrl_SUB;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_SUB;
    // rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_cmd_ack_SUB;
    rclcpp::Subscription<ros2_msgs::msg::FusePerception>::SharedPtr fuse_perception_SUB;


    // Stored SharedPtr
    ros2_msgs::msg::ControlInterface::SharedPtr last_final_ctrl = nullptr;

    // Variables
    uint8_t offboard_stream_counter = 0;
    bool arming_state = false;
    bool offboard_state = false;
    uint8_t last_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    bool control_state = false;
    Eigen::Quaternionf last_q = {1, 0, 0, 0};
    float yaw_W = 0;
    float odo_z_velocity = 0;

    enum class OffboardMode {
        POSITION,
        VELOCITY,
        ACCELERATION,
        ATTITUDE,
        RATE,
        THRUST,
        ACTUATOR
    };
    OffboardMode offboard_mode = OffboardMode::VELOCITY;
    enum class NodeLoopState {
        INIT, // Make stable stream of offboard topics
        ARM, // Send arm command after haveing a long enough stable stream
        OFFBOARD, // Running in offboard mode 
        LOST, // Lost input topic, hover still
        DISARM, // End offboard mode
    };
    NodeLoopState current_loop_state = NodeLoopState::INIT;
    bool just_change_loop_state = true;

    //Timer
    rclcpp::TimerBase::SharedPtr node_loop_TIME;

    // Callbacks
    void FinalCtrlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg);
    void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void FusePerceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg);
    void NodeLoopCallback();

    // Methods
    void PublishVehicleCmd(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    bool SendArmCmd();
    bool SendDisarmCmd();
    void SendOffboardCmd();
    void PublishOffboardControlMode();
    void PublishTrajectorySetpoint();
    void PublishAttitudeSetPoint();
};