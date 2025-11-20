#include "fuse_perception/fuse_perception.hpp"

FusePerceptionNode::FusePerceptionNode() : rclcpp::Node("fuse_perception") {
    using namespace std::chrono_literals;

    // Create Publisher
    fuse_PUB = this->create_publisher<ros2_msgs::msg::FusePerception>("/on_drone/sensor/fuse_perception", rclcpp::SensorDataQoS());

    // Create Subscriber
    odo_SUB = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::OdoCallback, this, _1)
    );

    contact_SUB = this->create_subscription<ros2_msgs::msg::ContactSensor>(
        "/on_drone/sensor/contact",
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::ContactCallback, this, _1)
    );

    scan_down_SUB = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "sensor/lidar_1d_down/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::ScanDownCallback, this, _1)
    );

    // Create wall timers
    publish_TIM = this->create_wall_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_NANOSEC),
        std::bind(&FusePerceptionNode::PublishCallback, this)
    );
}

FusePerceptionNode::~FusePerceptionNode() {}

float FusePerceptionNode::handleScanDown(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if(scan_down_init) {
        scan_down_range_min = msg->range_min;
        scan_down_range_max = msg->range_max;
        scan_down_init = false;
    }
    if(msg->ranges[0] < scan_down_range_min || msg->ranges[0] > scan_down_range_max) {
        return 0; // 0 mean CLEAR
    }
    else return msg->ranges[0];
}
/**
 * @brief Parse all data in last_ into the msg and publish
 */
void FusePerceptionNode::PublishCallback() {
    auto msg = ros2_msgs::msg::FusePerception();
    
    // Odometry
    if(last_odo != nullptr) {
        msg.frame = last_odo->pose_frame; // auto parse from PX4 to ROS2 frame
        msg.position = frame_utils::frameNEDtoENU(last_odo->position);
        msg.q = frame_utils::quaternionToArray(frame_utils::quaternionNEDtoENU(frame_utils::arrayToQuaternion(last_odo->q)));
        msg.velocity = frame_utils::frameNEDtoENU(last_odo->velocity);
        msg.angular_velocity = frame_utils::frameFRDtoFLU(last_odo->angular_velocity);

        last_odo = nullptr;
    }
    else {
        msg.frame = 0;
        msg.position.fill(NO_DATA_f);
        msg.q.fill(NO_DATA_f);
    }
    
    // Contact sensor
    if(last_contact != nullptr) {
        msg.bearable_contact = last_contact->bearable_contact;
        msg.critical_contact = last_contact->critical_contact;

        last_contact = nullptr;
    }
    else {
        msg.bearable_contact = false;
        msg.critical_contact = false;
    }

    // Scan down
    if(last_scan_down != nullptr) {
        msg.below_distance = handleScanDown(last_scan_down);
        
        last_scan_down = nullptr;
    }
    else {
        msg.below_distance = NO_DATA_f;
    }
    
    msg.header.stamp = this->get_clock()->now();
    fuse_PUB->publish(msg);
    RCLCPP_INFO(this->get_logger(), GREEN "Published" RESET);
}

void FusePerceptionNode::OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    last_odo = msg;
}

void FusePerceptionNode::ContactCallback(const ros2_msgs::msg::ContactSensor::SharedPtr msg) {
    last_contact = msg;
}

void FusePerceptionNode::ScanDownCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_down = msg;
}
