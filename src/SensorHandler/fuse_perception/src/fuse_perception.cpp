#include "fuse_perception/fuse_perception.hpp"

FusePerceptionNode::FusePerceptionNode() : rclcpp::Node("fuse_perception") {
    using namespace std::chrono_literals;

    Global::setup_for_simulation(this);

    // Tf 
    broadcaster_TF = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Create Publisher
    fuse_PUB = this->create_publisher<alpha_msgs::msg::FusePerception>(Topic::FUSE_PERCEPTION, rclcpp::SensorDataQoS());

    // Create Subscriber
    odo_SUB = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::OdoCallback, this, _1)
    );

    contact_SUB = this->create_subscription<alpha_msgs::msg::ContactSensor>(
        Topic::CONTACT_PARSER,
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::ContactCallback, this, _1)
    );

    scan_down_SUB = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "sensor/lidar_1d_down/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::ScanDownCallback, this, _1)
    );

    // Create wall timers
    publish_TIM = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FusePerceptionNode::PublishCallback, this)
    );

    // Init variable
    lost_lidar_down = true;
    missed_lidar_down = Threshold::MISSED_FAST_TOPIC;
    lost_odometry = true;
    missed_odometry = Threshold::MISSED_FAST_TOPIC;
    lidar_down_range_min = 0.1f;
    lidar_down_range_max = 30.0f;
}

FusePerceptionNode::~FusePerceptionNode() {}

float FusePerceptionNode::handleScanDown(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if(msg == nullptr) return NO_DATA_f;
    if(msg->ranges[0] < lidar_down_range_min || msg->ranges[0] > lidar_down_range_max) {
        return 0; // 0 mean CLEAR
    }
    else return msg->ranges[0];
}

void FusePerceptionNode::doFrameTransform(alpha_msgs::msg::FusePerception msg) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "world";
    tf.child_frame_id = base_link.get();
    tf.transform.translation.x = msg.position[0];
    tf.transform.translation.y = msg.position[1];
    tf.transform.translation.z = msg.position[2];
    tf.transform.rotation.w = msg.q[0];
    tf.transform.rotation.x = msg.q[1];
    tf.transform.rotation.y = msg.q[2];
    tf.transform.rotation.z = msg.q[3];
    broadcaster_TF->sendTransform(tf);
}

/**
 * @brief Parse all data in last_ into the msg and publish
 */
void FusePerceptionNode::PublishCallback() {
    auto msg = alpha_msgs::msg::FusePerception();
    
    // Odometry - stream
    if(missed_odometry < Threshold::MISSED_FAST_TOPIC) missed_odometry++;
    if(missed_odometry >= Threshold::MISSED_FAST_TOPIC) {
        if(lost_odometry == false) RCLCPP_WARN(this->get_logger(), YELLOW "Lost Odometry" RESET);
        lost_odometry = true;
        last_odo = nullptr;
    }else {
        if(lost_odometry == true) RCLCPP_WARN(this->get_logger(), GREEN "Received Odometry" RESET);
        lost_odometry = false;
    }

    if(!lost_odometry) {
        msg.frame = last_odo->pose_frame; // auto parse from PX4 to ROS2 frame
        msg.position = frame_utils::frameNEDtoENU(last_odo->position);
        msg.q = frame_utils::quaternionToArray(frame_utils::quaternionNEDtoENU(frame_utils::arrayToQuaternion(last_odo->q)));
        msg.velocity = frame_utils::frameNEDtoENU(last_odo->velocity);
        msg.angular_velocity = frame_utils::frameFRDtoFLU(last_odo->angular_velocity);

        float speed = std::sqrt(msg.velocity[0]*msg.velocity[0] + msg.velocity[1]*msg.velocity[1] + msg.velocity[2]*msg.velocity[2]);
        float hazard_distance = Drone::HAZARD_DISTANCE + speed * Drone::REACT_TIME + ((speed * speed) / (2 * Drone::DECELERATE_MAX));
        msg.hazard_distance_sq = hazard_distance*hazard_distance;

        doFrameTransform(msg);
    }
    else {
        msg.frame = 0;
        msg.position.fill(NO_DATA_f);
        msg.q.fill(NO_DATA_f);
        msg.velocity.fill(NO_DATA_f);
        msg.angular_velocity.fill(NO_DATA_f);
    }
    
    // Contact sensor - flag
    if(last_contact != nullptr) {
        msg.bearable_contact = last_contact->bearable_contact;
        msg.critical_contact = last_contact->critical_contact;

        last_contact = nullptr;
    }
    else {
        msg.bearable_contact = false;
        msg.critical_contact = false;
    }

    // Scan down - stream
    if(missed_lidar_down < Threshold::MISSED_FAST_TOPIC) missed_lidar_down++;
    if(missed_lidar_down >= Threshold::MISSED_FAST_TOPIC) {
        if(lost_lidar_down == false) RCLCPP_WARN(this->get_logger(), YELLOW "Lost lidar scan down" RESET);
        last_scan_down = nullptr;
        lost_lidar_down = true;

    }
    else {
        if(lost_lidar_down == true) RCLCPP_WARN(this->get_logger(), GREEN "Received lidar scan donw" RESET);
        lost_lidar_down = false;
    }
    msg.below_distance = handleScanDown(last_scan_down);
    
    // Publish msg
    msg.header.stamp = this->get_clock()->now();
    fuse_PUB->publish(msg);
    RCLCPP_INFO(this->get_logger(), GREEN "Published" RESET);
}

void FusePerceptionNode::OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    last_odo = msg;
    missed_odometry = 0;
}

void FusePerceptionNode::ContactCallback(const alpha_msgs::msg::ContactSensor::SharedPtr msg) {
    last_contact = msg;
}

void FusePerceptionNode::ScanDownCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_down = msg;
    if(missed_lidar_down >= Threshold::MISSED_FAST_TOPIC) {
        lidar_down_range_min = msg->range_min;
        lidar_down_range_max = msg->range_max;
    }
    missed_lidar_down = 0;
}
