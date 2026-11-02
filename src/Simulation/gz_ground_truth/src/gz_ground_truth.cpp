#include "gz_ground_truth/gz_ground_truth.hpp"

using std::placeholders::_1;

GzGroundTruthNode::GzGroundTruthNode() : rclcpp::Node("gz_ground_truth") {
    Global::setup_for_simulation(this);

    //: Create subscriber 
    this->SUB_gz_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        Topic::DRONE_ODOMETRY,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&GzGroundTruthNode::GzOdometryCallback, this, _1)
    );

    //: Create publisher 
    this->PUB_px4_odometry = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        Topic::Px4::VEHICLE_ODOMETRY,
        rclcpp::SystemDefaultsQoS()
    );

    //: Create timer 
    this->TIME_publish_px_odo = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&GzGroundTruthNode::PublishOdometryCallback, this)
    );

    //: Init Data 
    this->last_gz_odometry = nullptr;
    this->missed_gz_odometry = 0;
}

GzGroundTruthNode::~GzGroundTruthNode() {

}

void GzGroundTruthNode::GzOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->last_gz_odometry = msg;
    this->missed_gz_odometry = 0;
}

void GzGroundTruthNode::PublishOdometryCallback() {
    // Handle null and time out 
    if(missed_gz_odometry <= Threshold::ALLOW_MISSED_FAST_TOPIC) missed_gz_odometry++;
    else last_gz_odometry = nullptr;
    if(last_gz_odometry == nullptr) return;

    // Publish msg
    auto msg = px4_msgs::msg::VehicleOdometry();

    msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    std::array<float, 3> gz_position({
        static_cast<float>(last_gz_odometry->pose.pose.position.x),
        static_cast<float>(last_gz_odometry->pose.pose.position.y),
        static_cast<float>(last_gz_odometry->pose.pose.position.z)
    });
    msg.position = frame_utils::frameENUtoNED(gz_position);

    std::array<float, 4> gz_quad({
        static_cast<float>(last_gz_odometry->pose.pose.orientation.w),
        static_cast<float>(last_gz_odometry->pose.pose.orientation.x),
        static_cast<float>(last_gz_odometry->pose.pose.orientation.y),
        static_cast<float>(last_gz_odometry->pose.pose.orientation.z)
    });
    msg.q = frame_utils::quaternionENUtoNED(gz_quad);

    msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    std::array<float, 3> gz_linear_v({
        static_cast<float>(last_gz_odometry->twist.twist.linear.x),
        static_cast<float>(last_gz_odometry->twist.twist.linear.y),
        static_cast<float>(last_gz_odometry->twist.twist.linear.z)
    });
    msg.velocity = frame_utils::frameENUtoNED(gz_linear_v);

    std::array<float, 3> gz_angular_v({
        static_cast<float>(last_gz_odometry->twist.twist.angular.x),
        static_cast<float>(last_gz_odometry->twist.twist.angular.y),
        static_cast<float>(last_gz_odometry->twist.twist.angular.z)
    });
    msg.angular_velocity = frame_utils::frameFLUtoFRD(gz_angular_v);

    msg.timestamp = 
        last_gz_odometry->header.stamp.sec * 1'000'000ull + 
        last_gz_odometry->header.stamp.nanosec / 1'000ull;

    msg.reset_counter = 15;

    PUB_px4_odometry->publish(msg);
}