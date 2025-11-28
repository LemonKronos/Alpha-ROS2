#pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "ros2_msgs/msg/contact_sensor.hpp"
#include "ros_gz_interfaces/msg/contact.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

using std::placeholders::_1;

constexpr uint64_t PUBLISH_CYCLE = 1e8;

class ContactParserNode : public rclcpp::Node {
public:
    ContactParserNode();
    ~ContactParserNode();
private:
    // Publisher
    rclcpp::Publisher<ros2_msgs::msg::ContactSensor>::SharedPtr contact_PUB;

    // Subscriber
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_body_SUB;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_rotor0_SUB;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_rotor1_SUB;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_rotor2_SUB;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_rotor3_SUB;

    // Variables
    bool bearable = false;
    bool last_bearable = false;
    bool critical = false;
    bool last_critical = false;

    // Timers
    rclcpp::TimerBase::SharedPtr Contact_pub_TIME;

    // Callbacks
    void BodyCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr);
    void RotorCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr);
    void PublisherCallback();

};