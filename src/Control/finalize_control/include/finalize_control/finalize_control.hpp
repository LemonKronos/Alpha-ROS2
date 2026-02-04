#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "ros2_msgs/msg/control_interface.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

#include <algorithm>

constexpr float ATTITUDE_THRESHOLD = 0.01f;
constexpr uint8_t LOSS_FINAL_CONTROL_THRESHOLD = 8;
constexpr uint8_t OFFBOARD_STREAM_THRESHOLD = 20;
constexpr uint8_t OFFBOARD_MODE_CHANGE_THRESHOLD = OFFBOARD_STREAM_THRESHOLD / 2;

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
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_SUB;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_SUB;
    // rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_cmd_ack_SUB;

    // Stored SharedPtr
    ros2_msgs::msg::ControlInterface::SharedPtr last_final_ctrl = nullptr;

    // Variables
    uint8_t loss_final_control_count = 0;
    uint8_t offboard_stream_counter = 0;
    uint8_t offboard_attitude_counter = 0;
    bool arming_state = false;
    bool offboard_state = false;
    bool in_failure = false;
    bool landed = true;
    uint8_t last_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    bool control_state = false;
    Eigen::Quaternionf last_q = {1, 0, 0, 0};
    float yaw_W = 0;

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
    void VehicleLandedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void OdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void NodeLoopCallback();

    // Methods
    void PublishVehicleCmd(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    bool SendArmCmd();
    bool SendDisarmCmd();
    bool SendForceDisarmCmd();
    void SendOffboardCmd();
    void PublishOffboardControlMode();
    void PublishTrajectorySetpoint();
    void PublishAttitudeSetPoint();
};