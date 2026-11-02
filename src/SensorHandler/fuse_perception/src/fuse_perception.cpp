#include "fuse_perception/fuse_perception.hpp"

using std::placeholders::_1;

FusePerceptionNode::FusePerceptionNode() : rclcpp::Node("fuse_perception") {
    using namespace std::chrono_literals;

    Global::setup_for_simulation(this);

    //: Transform broadcaster 
    broadcaster_TF = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    //: Create Publisher 
    fuse_PUB = this->create_publisher<alpha_msgs::msg::FusePerception>(Topic::FUSE_PERCEPTION, rclcpp::SensorDataQoS());

    //: Create Subscriber 
    odo_SUB = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        Topic::Px4::VEHICLE_ODOMETRY,
        rclcpp::SensorDataQoS(),
        std::bind(&FusePerceptionNode::OdoCallback, this, _1)
    );

    //: Create wall timers 
    publish_TIM = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FusePerceptionNode::PublishCallback, this)
    );

    //: Init data 
    lost_odometry = true;
    missed_odometry = Threshold::ALLOW_MISSED_FAST_TOPIC;
    last_hazard_distance = Drone::HAZARD_DISTANCE;
}

FusePerceptionNode::~FusePerceptionNode() {}

void FusePerceptionNode::doFrameTransform(alpha_msgs::msg::FusePerception msg) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp.sec = last_odo->timestamp / 1'000'000ull;
    tf.header.stamp.nanosec = (last_odo->timestamp % 1'000'000ull) * 1'000ull;
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

void FusePerceptionNode::PublishCallback() {
    auto msg = alpha_msgs::msg::FusePerception();
    
    // Odometry - stream
    if(missed_odometry < Threshold::ALLOW_MISSED_FAST_TOPIC) missed_odometry++;
    if(missed_odometry >= Threshold::ALLOW_MISSED_FAST_TOPIC) {
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

        doFrameTransform(msg);
    }
    else {
        msg.frame = 0;
        msg.position.fill(NO_DATA_f);
        msg.q.fill(NO_DATA_f);
        msg.velocity.fill(NO_DATA_f);
        msg.angular_velocity.fill(NO_DATA_f);
    }
    
    // Publish msg
    msg.header.stamp = this->get_clock()->now();
    fuse_PUB->publish(msg);
    RCLCPP_INFO(this->get_logger(), GREEN "Published" RESET);
}

void FusePerceptionNode::OdoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    last_odo = msg;
    missed_odometry = 0;
}

