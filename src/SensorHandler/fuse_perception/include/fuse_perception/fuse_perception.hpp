# pragma once

#include "rclcpp/rclcpp.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "alpha_msgs/msg/fuse_perception.hpp"

class FusePerceptionNode : public rclcpp::Node {
public:
    FusePerceptionNode();
    ~FusePerceptionNode();

private:
    //: Frame transform 
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_TF;

    //: Publisher 
    rclcpp::Publisher<alpha_msgs::msg::FusePerception>::SharedPtr fuse_PUB;

    //: Subscriber 
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odo_SUB;

    //: Stored pointer 
    px4_msgs::msg::VehicleOdometry::SharedPtr last_odo = nullptr;

    //: Data 
    Name::Dynamic::BASE_LINK base_link;
    bool lost_odometry;
    uint8_t missed_odometry;
    float last_hazard_distance;

    //: Timer 
    rclcpp::TimerBase::SharedPtr publish_TIM;

    //: Method 
    void doFrameTransform(alpha_msgs::msg::FusePerception msg);
    
    //: Callback 
    void OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void PublishCallback();

};