#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"

class GzGroundTruthNode : public rclcpp::Node {
public:
    GzGroundTruthNode();
    ~GzGroundTruthNode();

private:
    //: Subscriber 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SUB_gz_odometry;

    //: Publisher 
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr PUB_px4_odometry;

    //: Timer
    rclcpp::TimerBase::SharedPtr TIME_publish_px_odo;

    //: Callback
    void GzOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void PublishOdometryCallback();

    //: Data 
    nav_msgs::msg::Odometry::SharedPtr last_gz_odometry = nullptr;
    uint8_t missed_gz_odometry = 0;
};