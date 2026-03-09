# pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ros2_msgs/msg/contact_sensor.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"

using std::placeholders::_1;

class FusePerceptionNode : public rclcpp::Node {
public:
    FusePerceptionNode();
    ~FusePerceptionNode();

private:
    // Tf 
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_TF;

    // Publisher
    rclcpp::Publisher<ros2_msgs::msg::FusePerception>::SharedPtr fuse_PUB;

    // Subscriber
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odo_SUB;
    rclcpp::Subscription<ros2_msgs::msg::ContactSensor>::SharedPtr contact_SUB;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_down_SUB;

    // Stored topic Shared pointer
    px4_msgs::msg::VehicleOdometry::SharedPtr last_odo = nullptr;
    ros2_msgs::msg::ContactSensor::SharedPtr last_contact = nullptr;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_down = nullptr;

    // Variales
    bool lost_lidar_down;
    uint8_t missed_lidar_down;
    bool lost_odometry;
    uint8_t missed_odometry;
    float lidar_down_range_min;
    float lidar_down_range_max;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_TIM;

    // Methods
    float handleScanDown(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void doFrameTransform(ros2_msgs::msg::FusePerception msg);
    
    // Callbacks
    void OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void ContactCallback(const ros2_msgs::msg::ContactSensor::SharedPtr msg);
    void ScanDownCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void PublishCallback();

};