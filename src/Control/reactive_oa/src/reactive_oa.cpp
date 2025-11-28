#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    RCLCPP_INFO(this->get_logger(), "Reactive OA Node has been started.");

    setup_for_simulation(this);
    
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
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );

    // Init variables
    obstacle_encountered = false;
    control_vec = Eigen::Vector3f::Zero();
    control_angular_vec = Eigen::Vector3f::Zero();
    movement_vec = Eigen::Vector3f::Zero();
    movement_angular_vec = Eigen::Vector3f::Zero();
    repulsive_vec = Eigen::Vector3f::Zero();
    repulsive_damp_unit = 0;
    correction_vec = Eigen::Vector3f::Zero();
    correction_angular_vec = Eigen::Vector3f::Zero();
    obstacle_clear_counter = 0;
}
ReactiveOANode::~ReactiveOANode(){}

/*################################################# Methods*/

void ReactiveOANode::computeControlVector(){
    control_vec = Eigen::Vector3f(
        last_input_control->forward * SPEED_MAX_FORWARD,
        last_input_control->left * SPEED_MAX_FORWARD,
        0
    ); // body frame

    control_angular_vec = Eigen::Vector3f(
        last_input_control->roll,
        last_input_control->pitch,
        last_input_control->yaw
    ); // body frame
}

void ReactiveOANode::computeMovementVector(){
    movement_vec = Eigen::Vector3f(
        last_perception->velocity[0],
        last_perception->velocity[1],
        0
    ); // world frame
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_perception->q);
    movement_vec = q.inverse() * movement_vec; // body frame

    movement_angular_vec = Eigen::Vector3f(
        last_perception->angular_velocity[0],
        last_perception->angular_velocity[1],
        last_perception->angular_velocity[2]
    ); // world frame
    movement_angular_vec = q.inverse() * movement_angular_vec; // body frame
}

void ReactiveOANode::computeRepulsiveVector(){
    // Compute repulsive vector by combining all obstacle points' influence
    repulsive_vec = Eigen::Vector3f::Zero();
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(const auto& contour : obstacle.getContours(index)) {
            for(const auto& point : contour.getContour()) {
                Eigen::Vector3f point_vec(point.x, point.y, 0);
                const float factor = (safe_distance - point_vec.norm()); // Cut depth magnitude
                point_vec = point_vec.normalized() * factor;
                repulsive_vec -= point_vec;
            }
        }
    }

    // Snap to 0
    if(repulsive_vec.isZero(1e-4f)) {
        repulsive_vec.setZero();
        return;
    }

    // Scale repulsive vector to try to maintain safe bubble
    const float x = (safe_distance - obstacle.getMinDistance()) / safe_distance;
    // const float scale = x * x * (3 - 2 * x); // Smooth S scale
    // const float scale = sqrtf(x);
    const float scale = 2.0f * x; // Linear is best
    repulsive_vec = repulsive_vec.normalized() * (scale * SPEED_MAX_FORWARD);
}

void ReactiveOANode::computeCorrectionVector() {
    correction_vec = control_vec + repulsive_vec;

    // Ensure correction vector is at most parallel to obstacle
    if (!repulsive_vec.isZero(1e-4f)) {
        Eigen::Vector3f repulsive_dir = repulsive_vec.normalized();
        float dot = repulsive_dir.dot(correction_vec);
        if (dot < 0.0f) {
            correction_vec -= dot * repulsive_dir;
        }
    }

    // Clamp correction vector to max speed
    correction_vec = Eigen::Vector3f(
        std::clamp(correction_vec.x(), -SPEED_MAX_FORWARD, SPEED_MAX_FORWARD),
        std::clamp(correction_vec.y(), -SPEED_MAX_FORWARD, SPEED_MAX_FORWARD),
        0
    );
    correction_angular_vec = control_angular_vec;
}

void ReactiveOANode::resetVectors(){
    control_vec = Eigen::Vector3f::Zero();
    control_angular_vec = Eigen::Vector3f::Zero();
    movement_vec = Eigen::Vector3f::Zero();
    movement_angular_vec = Eigen::Vector3f::Zero();

    if(obstacle_clear_counter == 1) repulsive_damp_unit = repulsive_vec.norm() / OBSTACLE_CLEAR_COUNT_THRESHOLD;
    if(obstacle_clear_counter >= OBSTACLE_CLEAR_COUNT_THRESHOLD) {
        repulsive_vec = Eigen::Vector3f::Zero();
    }
    else {
        const float scale = repulsive_vec.norm() - repulsive_damp_unit * obstacle_clear_counter;
        repulsive_vec = repulsive_vec.normalized() * scale;
    }

    correction_vec = Eigen::Vector3f::Zero();
    correction_angular_vec = Eigen::Vector3f::Zero();
}

#ifdef VISUALIZE
void ReactiveOANode::publishVectorArrow(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const Eigen::Vector3f& vec,
    float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
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
    computeCorrectionVector();
    auto msg = ros2_msgs::msg::ControlInterface();
    msg.control_state = true;
    
    if(obstacle_encountered) msg.control_by = ros2_msgs::msg::ControlInterface::REACTIVE_OA;
    else {
        msg.control_by = ros2_msgs::msg::ControlInterface::HUMAN;
        if(obstacle_clear_counter < OBSTACLE_CLEAR_COUNT_THRESHOLD) {
            obstacle_clear_counter++;
        }
    }
    obstacle_encountered = false;
    
    msg.forward = correction_vec.x() / SPEED_MAX_FORWARD;
    msg.left = correction_vec.y() / SPEED_MAX_FORWARD;
    // msg.up = correction_vec.z() / SPEED_MAX_UP_FW; // Still in 2D

    if(last_input_control != nullptr) { // Still meaningful for changing mode, etc.
        if(!last_input) {
            RCLCPP_INFO(this->get_logger(), GREEN "Received control input." RESET);
            last_input = true;
        }
        msg.roll = last_input_control->roll;
        msg.pitch = last_input_control->pitch;
        msg.yaw = last_input_control->yaw; // Still in 2D
        msg.up = last_input_control->up; // Still in 2D
        msg.wings_mode = last_input_control->wings_mode;
    }
    else {
        if(last_input) {
            RCLCPP_INFO(this->get_logger(), YELLOW "Waiting for control input." RESET);
            last_input = false;
        }
    }

    msg.header.stamp = this->get_clock()->now();
    final_control_PUB->publish(msg);
    
    #ifdef VISUALIZE
    publishVectorArrow(control_vec_PUB, control_vec, 0.0, 0.0, 1.0); // Blue
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0, 0.7, 0.7); // Teal
    publishVectorArrow(repulsive_vec_PUB, repulsive_vec, 1.0, 0.75, 0.75); // Pink
    publishVectorArrow(correction_vec_PUB, correction_vec, 1.0, 0.0, 0.0); // Red
    #endif

    last_input_control = nullptr;
    resetVectors();
}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg){
    last_input_control = msg;
    computeControlVector();
}

void ReactiveOANode::closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_encountered = true;
    obstacle_clear_counter = 0;
    obstacle.topicToObstacle(msg);
    safe_distance = obstacle.safe_distance;
    computeRepulsiveVector();
}

void ReactiveOANode::perceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg){
    last_perception = msg;
    computeMovementVector();
}
