# pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "ros2_msgs/msg/contact_sensor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ros2_msgs/msg/fuse_perception.hpp"

using std::placeholders::_1;

class FusePerceptionNode : public rclcpp::Node {
public:
    FusePerceptionNode();
    ~FusePerceptionNode();

private:
    // Publisher
    rclcpp::Publisher<ros2_msgs::msg::FusePerception>::SharedPtr fuse_PUB;

    // Subscriber
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odo_SUB;
    rclcpp::Subscription<ros2_msgs::msg::ContactSensor>::SharedPtr contact_SUB;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_down_SUB;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_cam_SUB;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_cam_SUB;

    // Stored topic Shared pointer
    px4_msgs::msg::VehicleOdometry::SharedPtr last_odo = nullptr;
    ros2_msgs::msg::ContactSensor::SharedPtr last_contact = nullptr;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_down = nullptr;
    sensor_msgs::msg::Image::SharedPtr last_depth_cam = nullptr;
    sensor_msgs::msg::Image::SharedPtr last_rgb_cam = nullptr;

    // Variales
    bool scan_down_init = true;
    float scan_down_range_min = 0.1f;
    float scan_down_range_max = 50.0f;
    // Timers
    rclcpp::TimerBase::SharedPtr publish_TIM;

    // Methods
    float handleScanDown(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Callbacks
    void OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void ContactCallback(const ros2_msgs::msg::ContactSensor::SharedPtr msg);
    void ScanDownCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void DepthCamCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void RGBCamCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void PublishCallback();

};