#pragma once

#include <rclcpp/rclcpp.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <std_msgs/msg/float64.hpp>
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/actuator_servos.hpp"

#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

//: Local dev config
#define TIME_ANALYSE (ALLOW_DEBUG & 0)
#define FLOW (ALLOW_DEBUG & 0)

class Px4EmulatorNode : public rclcpp::Node {
public:
    Px4EmulatorNode();
    ~Px4EmulatorNode();

private:
    //: Subscriber
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr SUB_actuator_motors;
    rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr SUB_actuator_servos;
    
    //: Publisher
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr PUB_vehicle_status;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr PUB_actuator_motors;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PUB_actuator_servo_0;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PUB_actuator_servo_1;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PUB_actuator_servo_2;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr PUB_actuator_servo_3;

    //: Timmer
    rclcpp::TimerBase::SharedPtr TIME_response_loop;
    
    //: Callback
    void VehicleCommandCallback();
    void ActuatorMotorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg);
    void ActuatorServosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr msg);

    //: Test
    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif
};