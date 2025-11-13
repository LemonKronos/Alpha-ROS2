#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    RCLCPP_INFO(this->get_logger(), "Reactive OA Node has been started.");

    // Publisher
    final_control_PUB = this->create_publisher<ros2_msgs::msg::ControlInterface>("control/final", 10);

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

void ReactiveOANode::computeAvoidanceVector(){
    Eigen::Vector3f repulsive_vec(0, 0, 0);
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(auto contour : obstacle.getObstacles(index)) {
            for( auto point : contour.getContour()) {
                Eigen::Vector3f point_vec(point.x, point.y, 0);
                float dist = point_vec.norm();
                if(dist < safe_distance && dist > 1e-6) {
                    point_vec = -point_vec.normalized() * (safe_distance - dist);
                }
                repulsive_vec += point_vec;
            }
        }
    }
    repulsive_vec = repulsive_vec.normalized() * control_vec.norm();
    avoidance_vec = control_vec + repulsive_vec;
}

void ReactiveOANode::computeMovementVector(){
    control_vec = Eigen::Vector3f(
        last_perception->velocity[0],
        last_perception->velocity[1],
        last_perception->velocity[2]
    ); // world frame
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_perception->q);
    control_vec = q * control_vec; // body frame

    control_angular_vec = Eigen::Vector3f(
        last_perception->angular_velocity[0],
        last_perception->angular_velocity[1],
        last_perception->angular_velocity[2]
    ); // world frame
    control_angular_vec = q * control_angular_vec; // body frame
}

void ReactiveOANode::computeSafeDistance(){
    float speed = control_vec.norm();
    safe_distance = HAZARD_DISTANCE + speed * REACT_TIME + ((speed * speed) / (2 * DECELERATE_MAX));
}

/*################################################# node loop */

void ReactiveOANode::nodeLoopCallback() {
    if(last_input_control == nullptr) {
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
    if(obstacle_encountered) { // Use avoidance vector
        msg.control_by = ros2_msgs::msg::ControlInterface::REACTIVE_OA;
        msg.control_state = true;

        msg.roll = last_input_control->roll;
        msg.pitch = last_input_control->pitch;
        msg.yaw = last_input_control->yaw;

        msg.forward = avoidance_vec.x();
        msg.right = avoidance_vec.y();
        msg.down = last_input_control->down;

        msg.wings_mode = last_input_control->wings_mode;
        RCLCPP_WARN(this->get_logger(), PINK "Reactive OA in action" RESET);
        obstacle_encountered = false;
    }
    else { // Just parse input control
        msg = *last_input_control;
    }
    msg.header.stamp = this->get_clock()->now();
    final_control_PUB->publish(msg);
    last_input_control = nullptr;
}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg){
    last_input_control = msg;
}

void ReactiveOANode::closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_encountered = true;
    obstacle = Obstacle();
    obstacle.topicToObstacle(msg->obstacles);
    computeAvoidanceVector();
}

void ReactiveOANode::perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg){
    last_perception = msg;
    computeMovementVector();
    computeSafeDistance();
}