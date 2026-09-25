#pragma once

#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

//: Local config
#define ALLOW_ATTITUDE true

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

//: Local dev config
#define TIME_ANALYSE (ALLOW_DEBUG & 0)

constexpr float ATTITUDE_THRESHOLD = 0.01f;
constexpr uint8_t LOSS_FINAL_CONTROL_THRESHOLD = 8;
constexpr uint8_t OFFBOARD_STREAM_THRESHOLD = 20;
constexpr uint8_t OFFBOARD_MODE_CHANGE_THRESHOLD = OFFBOARD_STREAM_THRESHOLD / 2;

using std::placeholders::_1;
using std::mutex;
using std::lock_guard;
using std::atomic;

using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleOdometry;
// using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::TrajectorySetpoint;
using alpha_msgs::msg::ControlInterface;

class FinalizeControlNode : public rclcpp::Node {
public:
    FinalizeControlNode();
    ~FinalizeControlNode();

private:
    //: Callback Group
    rclcpp::CallbackGroup::SharedPtr timer_group;
    rclcpp::CallbackGroup::SharedPtr msg_group;

    //: Publisher
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_cmd_PUB;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_PUB;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_set_PUB;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_set_PUB;

    //: Subcriber
    rclcpp::Subscription<ControlInterface>::SharedPtr input_control_SUB;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_SUB;
    rclcpp::Subscription<VehicleLandDetected>::SharedPtr vehicle_land_SUB;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_SUB;
    // rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_cmd_ack_SUB;

    //: Stored SharedPtr
    mutex mx_control;
    ControlInterface::SharedPtr latest_input_control;
    ControlInterface::SharedPtr input_control;

    //: Variables
    uint8_t loss_final_control_count;
    uint8_t offboard_stream_counter;
    uint8_t offboard_attitude_counter;
    atomic<bool> arming_state;
    atomic<bool> offboard_state;
    atomic<bool> in_failure;
    atomic<bool> landed;
    atomic<uint8_t> last_nav_state;
    bool control_state;
    atomic<bool> in_hover;
    
    mutex mx_spatial;
    std::array<float, 3> latest_position_NED;
    std::array<float, 3> position_NED;
    Eigen::Quaternionf latest_quaternion;
    Eigen::Quaternionf quaternion;

    float yaw_W;

    enum class OffboardMode {
        POSITION,
        VELOCITY,
        ACCELERATION,
        ATTITUDE,
        RATE,
        THRUST,
        ACTUATOR
    };
    atomic<OffboardMode> offboard_mode{OffboardMode::VELOCITY};

    enum class NodeLoopState {
        INIT, // Make stable stream of offboard topics
        ARM, // Send arm command after haveing a long enough stable stream
        OFFBOARD, // Running in offboard mode 
        LOST, // Lost input topic, hover still
        DISARM, // End offboard mode
    };
    NodeLoopState current_loop_state;
    bool just_change_loop_state;

    //: Timer
    rclcpp::TimerBase::SharedPtr node_loop_TIME;

    //: Callbacks
    void InputControlCallback(const ControlInterface::SharedPtr msg);
    void VehicleStatusCallback(const VehicleStatus::SharedPtr msg);
    void VehicleLandedCallback(const VehicleLandDetected::SharedPtr msg);
    void OdometryCallback(const VehicleOdometry::SharedPtr msg);
    void NodeLoopCallback();

    //: Methods
    void PublishVehicleCmd(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    bool SendArmCmd();
    bool SendDisarmCmd();
    bool SendForceDisarmCmd();
    void SendOffboardCmd();
    void PublishOffboardControlMode();
    void PublishTrajectorySetpoint();
    void PublishAttitudeSetPoint();

    //: Test
#if ALLOW_DEBUG
    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif
#endif
};