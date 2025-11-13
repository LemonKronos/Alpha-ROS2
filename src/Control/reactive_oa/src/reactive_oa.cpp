#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    RCLCPP_INFO(this->get_logger(), "Reactive OA Node has been started.");

    // Publisher
    final_control_PUB = this->create_publisher<ros2_msgs::msg::ControlInterface>("control/final", 10);

    #ifdef VISUALIZE
        control_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/control_vector", 10);
        movement_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/movement_vector", 10);
        repulsive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/repulsive_vector", 10);
        correction_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/correction_vector", 10);
    #endif

    // Subscriber
    input_control_SUB = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        "control/input",
        10,
        std::bind(&ReactiveOANode::inputControlCallback, this, _1));

    close_contour_SUB = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/close",
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::closeContourCallback, this, _1));

    perception_SUB = this->create_subscription<ros2_msgs::msg::FusePerception>(
        "/on_drone/sensor/fuse_perception",
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::perceptionCallback, this, _1));

    // Timer
    node_loop_TIME = this->create_wall_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );
}
ReactiveOANode::~ReactiveOANode(){}

/*################################################# Methods*/

void ReactiveOANode::computeControlVector(){
    control_vec = Eigen::Vector3f(
        last_input_control->forward * SPEED_MAX_FORWARD_FW,
        last_input_control->right * SPEED_MAX_STRAFE,
        0
    ); // body frame

    control_angular_vec = Eigen::Vector3f(
        last_input_control->roll,
        last_input_control->pitch,
        0
    ); // body frame
}

void ReactiveOANode::computeMovementVector(){
    movement_vec = Eigen::Vector3f(
        last_perception->velocity[0],
        last_perception->velocity[1],
        0
    ); // world frame
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_perception->q);
    movement_vec = q * movement_vec; // body frame

    movement_angular_vec = Eigen::Vector3f(
        last_perception->angular_velocity[0],
        last_perception->angular_velocity[1],
        0
    ); // world frame
    movement_angular_vec = q * movement_angular_vec; // body frame
}

void ReactiveOANode::computeRepulsiveVector(){
    repulsive_vec = Eigen::Vector3f::Zero();
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(auto contour : obstacle.getContours(index)) {
            for( auto point : contour.getContour()) {
                Eigen::Vector3f point_vec(point.x, point.y, 0);
                float dist = point_vec.norm();
                point_vec = - point_vec.normalized() * (safe_distance - dist);
                repulsive_vec += point_vec;
            }
        }
    }
    float control_magnitude = control_vec.norm();
    float movement_magnitude = movement_vec.norm();
    float scale = control_magnitude > movement_magnitude ? control_magnitude : movement_magnitude;
    float tune = 1.5f; // tune factor
    float min_escape_speed = (safe_distance) * tune;
    if(scale < min_escape_speed) {
        repulsive_vec = repulsive_vec.normalized() * min_escape_speed;
    }
    else repulsive_vec = repulsive_vec.normalized() * scale;
}

void ReactiveOANode::computeCorrectionVector(){
    correction_vec = repulsive_vec + control_vec;
    if(correction_vec.norm() > control_vec.norm()) {
        correction_vec = correction_vec.normalized() * control_vec.norm();
    }

}

void ReactiveOANode::computeSafeDistance(){
    speed_2d = movement_vec.head<2>().norm();
    safe_distance = HAZARD_DISTANCE + speed_2d * REACT_TIME + ((speed_2d * speed_2d) / (2 * DECELERATE_MAX));
}

#ifdef VISUALIZE
void ReactiveOANode::publishVectorArrow(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const Eigen::Vector3f& vec,
    float r, float g, float b)
{
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";   // or "map"/"odom" depending on your frame
    arrow.header.stamp = this->now();
    arrow.ns = "oa_vectors";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // Arrow scale (bigger arrow)
    arrow.scale.x = 0.5;  // shaft thickness
    arrow.scale.y = 0.8;   // head diameter
    arrow.scale.z = 0.5;   // head length

    // Color
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    // Define start and end points
    arrow.points.resize(2);
    arrow.points[0].x = 0.0;
    arrow.points[0].y = 0.0;
    arrow.points[0].z = 0.0;

    float viz_scale = 1.0f;  // make vector longer visually if needed
    arrow.points[1].x = vec.x() * viz_scale;
    arrow.points[1].y = vec.y() * viz_scale;
    arrow.points[1].z = vec.z() * viz_scale;

    pub->publish(arrow);
}
#endif

/*################################################# node loop */

void ReactiveOANode::nodeLoopCallback() {
    if(obstacle_encountered) {
        auto msg = ros2_msgs::msg::ControlInterface();
        msg.control_by = ros2_msgs::msg::ControlInterface::REACTIVE_OA;
        msg.control_state = true;
        msg.roll = 0.0f;
        msg.pitch = 0.0f;
        msg.yaw = 0.0f;
        msg.forward = repulsive_vec.x();
        msg.right = repulsive_vec.y();
        msg.down = repulsive_vec.z();
        msg.wings_mode = ros2_msgs::msg::ControlInterface::UNCHANGE;
        msg.header.stamp = this->get_clock()->now();
        final_control_PUB->publish(msg);
        RCLCPP_WARN(this->get_logger(), PINK "Reactive OA in action" RESET);
        obstacle_encountered = false;
    }
    else {
        if(last_input_control == nullptr) { // No input control
            if(last_input) {
                RCLCPP_INFO(this->get_logger(), YELLOW "Waiting for input control..." RESET);
                last_input = false;
            }
            return;
        }
        if(!last_input) {
            RCLCPP_INFO(this->get_logger(), GREEN "Received input control." RESET);
            last_input = true;
        }
        auto msg = ros2_msgs::msg::ControlInterface();
        msg = *last_input_control;
        msg.header.stamp = this->get_clock()->now();
        final_control_PUB->publish(msg);
    }
    last_input_control = nullptr;

    #ifdef VISUALIZE
        computeCorrectionVector();
        publishVectorArrow(control_vec_PUB, control_vec, 0.0, 0.0, 1.0); // Blue
        publishVectorArrow(movement_vec_PUB, movement_vec, 1.0, 1.0, 0.0); // Yellow
        publishVectorArrow(repulsive_vec_PUB, repulsive_vec, 1.0, 0.75, 0.75); // Pink
        publishVectorArrow(correction_vec_PUB, correction_vec, 0.0, 1.0, 0.0); // Green
    #endif
}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg){
    last_input_control = msg;
    computeControlVector();
}

void ReactiveOANode::closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_encountered = true;
    obstacle.topicToObstacle(msg);
    computeRepulsiveVector();
}

void ReactiveOANode::perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg){
    last_perception = msg;
    computeMovementVector();
    computeSafeDistance();
}