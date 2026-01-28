#include "contact_parser/contact_parser.hpp"

ContactParserNode::ContactParserNode() : rclcpp::Node("contact_parser") {
    using namespace std::chrono_literals;

    Global::setup_for_simulation(this);
    
    // Create Subscriber
    contact_body_SUB = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "sensor/contact_body/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&ContactParserNode::BodyCallback, this, _1)
    );

    contact_rotor0_SUB = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "sensor/contact_rotor0/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&ContactParserNode::RotorCallback, this, _1)
    );

    contact_rotor1_SUB = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "sensor/contact_rotor1/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&ContactParserNode::RotorCallback, this, _1)
    );

    contact_rotor2_SUB = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "sensor/contact_rotor2/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&ContactParserNode::RotorCallback, this, _1)
    );

    contact_rotor3_SUB = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        "sensor/contact_rotor3/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&ContactParserNode::RotorCallback, this, _1)
    );

    // Create Publisher
    contact_PUB = this->create_publisher<ros2_msgs::msg::ContactSensor>(Topic::CONTACT_PARSER, rclcpp::SensorDataQoS());

    // Create wall timer
    Contact_pub_TIME = this->create_timer(
        std::chrono::nanoseconds(PUBLISH_CYCLE),
        std::bind(&ContactParserNode::PublisherCallback, this)
    );

    // Init variables
    bearable = false;
    last_bearable = false;
    critical = false;
    last_critical = false;
};

ContactParserNode::~ContactParserNode() {

}

void ContactParserNode::BodyCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr) {
    bearable = true;
}

void ContactParserNode::RotorCallback(const ros_gz_interfaces::msg::Contacts::SharedPtr) {
    critical = true;
}

void ContactParserNode::PublisherCallback() {
    bool current_bearable = bearable;
    bool current_critical = critical;

    if(current_bearable != last_bearable || current_critical != last_critical) {
        auto msg = ros2_msgs::msg::ContactSensor();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.bearable_contact = current_bearable;
        msg.critical_contact = current_critical;

        contact_PUB->publish(msg);

        if(current_critical) RCLCPP_WARN(this->get_logger(), RED "Contact Sensor critical" RESET);
        else if(current_bearable) RCLCPP_WARN(this->get_logger(), YELLOW "Contact Sensor bearable" RESET);
        else RCLCPP_WARN(this->get_logger(), BLUE "Contact Sensor safe" RESET);

        last_bearable = current_bearable;
        last_critical = current_critical;
    }
    bearable = false;
    critical = false;
}
