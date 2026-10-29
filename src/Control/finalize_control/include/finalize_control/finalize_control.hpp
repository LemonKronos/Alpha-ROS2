#pragma once

#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"

#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "alpha_msgs/msg/control_interface.hpp"
#include "alpha_msgs/msg/normalized_motors_servos.hpp"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

//: Local dev config
#define TIME_ANALYSE (ALLOW_DEBUG & 0)
#define FLOW (ALLOW_DEBUG & 0)

class FinalizeControlNode : public rclcpp::Node {
public:
    FinalizeControlNode();
    ~FinalizeControlNode();

private:
    //: Callback Group
    rclcpp::CallbackGroup::SharedPtr control_signal_group;
    rclcpp::CallbackGroup::SharedPtr msg_callback_group;

    //: Publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr PUB_vehicle_cmd;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr PUB_offboard_ctrl;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr PUB_attitude_set;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr PUB_trajectory_set;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr PUB_motors;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr PUB_servos;

    //: Subcriber
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr SUB_raw_input_control;
    rclcpp::Subscription<alpha_msgs::msg::ControlInterface>::SharedPtr SUB_reactive_control;
    rclcpp::Subscription<alpha_msgs::msg::NormalizedMotorsServos>::SharedPtr SUB_acrobatic_control;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr SUB_vehicle_status;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr SUB_vehicle_land;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr SUB_odometry;
    // rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_cmd_ack_SUB;

    //: Stored SharedPtr
    std::mutex mutex_raw_input;
    alpha_msgs::msg::ControlInterface::SharedPtr last_raw_input_control;
    alpha_msgs::msg::ControlInterface::SharedPtr raw_input_control;

    std::mutex mutex_reactive;
    alpha_msgs::msg::ControlInterface::SharedPtr last_reactive_control;
    alpha_msgs::msg::ControlInterface::SharedPtr reactive_control;

    std::mutex mutex_acrobatic;
    alpha_msgs::msg::NormalizedMotorsServos::SharedPtr last_acrobatic_control;
    alpha_msgs::msg::NormalizedMotorsServos::SharedPtr acrobatic_control;

    //: Variables
    alpha_msgs::msg::ControlInterface::SharedPtr final_control;

    uint8_t offboard_stream_counter;
    uint8_t offboard_attitude_counter;
    std::atomic<bool> arming_state;
    std::atomic<bool> offboard_state;
    std::atomic<bool> in_failure;
    std::atomic<bool> landed;
    std::atomic<uint8_t> last_nav_state;
    bool control_state;
    std::atomic<bool> in_hover;
    
    std::mutex mutex_pose;
    std::array<float, 3> latest_position_NED;
    std::array<float, 3> position_NED;
    Eigen::Quaternionf latest_quaternion_NED;
    Eigen::Quaternionf quaternion_ENU;

    float heading_NED;
    float heading_ENU;

    uint8_t control_support = alpha_msgs::msg::ControlInterface::NONE;

    enum class OffboardMode {
        POSITION,
        VELOCITY,
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
    NodeLoopState current_loop_state;
    bool just_change_loop_state;

    //: Timer
    rclcpp::TimerBase::SharedPtr TIME_node_loop;

    //: Callbacks
    void RawInputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg);
    void ReactiveControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg);
    void AcrobaticControlCallback(const alpha_msgs::msg::NormalizedMotorsServos::SharedPtr msg);
    void VehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void VehicleLandedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void OdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void NodeLoopCallback();

    //: Methods
    void PublishVehicleCmd(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    bool SendArmCmd();
    bool SendDisarmCmd();
    bool SendForceDisarmCmd();
    
    void SendOffboardCmd();
    void PublishTrajectorySetpoint();
    void PublishAttitudeSetPoint();
    void PublishActuatorMotors();
    void PublishActuatorServos();

    //: Test
    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif
};