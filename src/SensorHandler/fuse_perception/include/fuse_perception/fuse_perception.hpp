# pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "alpha_msgs/msg/contact_sensor.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"

using std::placeholders::_1;

class FusePerceptionNode : public rclcpp::Node {
public:
    FusePerceptionNode();
    ~FusePerceptionNode();

private:
    // Tf 
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_TF;

    // Publisher
    rclcpp::Publisher<alpha_msgs::msg::FusePerception>::SharedPtr fuse_PUB;

    // Subscriber
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odo_SUB;
    rclcpp::Subscription<alpha_msgs::msg::ContactSensor>::SharedPtr contact_SUB;

    // Stored topic Shared pointer
    px4_msgs::msg::VehicleOdometry::SharedPtr last_odo = nullptr;
    alpha_msgs::msg::ContactSensor::SharedPtr last_contact = nullptr;

    // Variales
    Name::Dynamic::BASE_LINK base_link;
    bool lost_odometry;
    uint8_t missed_odometry;
    float last_hazard_distance;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_TIM;

    // Methods
    void doFrameTransform(alpha_msgs::msg::FusePerception msg);
    
    // Callbacks
    void OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void ContactCallback(const alpha_msgs::msg::ContactSensor::SharedPtr msg);
    void PublishCallback();

};