#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    
    setup_for_simulation(this);
    
    // Publisher
    final_control_PUB = this->create_publisher<ros2_msgs::msg::ControlInterface>(CONTROL_REACTIVE_TOPIC, 10);

    #ifdef VISUALIZE
        control_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/control_vector", 10);
        movement_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/movement_vector", 10);
        repulsive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/repulsive_vector", 10);
        correction_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/correction_vector", 10);
    #endif

    // Subscriber
    input_control_SUB = this->create_subscription<ros2_msgs::msg::ControlInterface>(
        CONTROL_INPUT_TOPIC,
        10,
        std::bind(&ReactiveOANode::inputControlCallback, this, _1));

    close_contour_SUB = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        LIDAR_2D_CONTOUR_CLOSE_TOPIC,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::closeContourCallback, this, _1));

    odo_SUB = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::odoCallback, this, _1));

    // Timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(SYSTEM_LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );

    // Init variables
    control_vec.setZero();
    control_angular_vec.setZero();
    movement_vec.setZero();
    movement_angular_vec.setZero();
    repulsive_vec.setZero();
    correction_vec.setZero();
    correction_angular_vec.setZero();

    repulsive_damping_counter = 0;
    obstacle_clear_damping_counter = 0;

    safe_distance = HAZARD_DISTANCE;

    lost_control_signal = true;
    lost_control_signal_counter = 0;
}
ReactiveOANode::~ReactiveOANode(){}

/*################################################# Methods*/

void ReactiveOANode::computeControlVector() {
    control_vec = Eigen::Vector3f(
        last_control_signal->forward * SPEED_MAX_FORWARD,
        last_control_signal->left * SPEED_MAX_FORWARD,
        0 // last_control_signal->up * SPEED_MAX_UP // Still in 2D
    ); // body frame

    control_angular_vec = Eigen::Vector3f(
        last_control_signal->roll,
        last_control_signal->pitch,
        last_control_signal->yaw
    ); // body frame
}

void ReactiveOANode::computeMovementVector() {
    movement_vec = Eigen::Vector3f(
        last_odo->velocity[0],
        last_odo->velocity[1],
        last_odo->velocity[2]
    ); // world frame
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_odo->q);
    movement_vec = q.inverse() * movement_vec; // body frame
    movement_vec(2) = 0; // Still in 2D

    movement_angular_vec = Eigen::Vector3f(
        last_odo->angular_velocity[0],
        last_odo->angular_velocity[1],
        last_odo->angular_velocity[2]
    ); // world frame
    movement_angular_vec = q.inverse() * movement_angular_vec; // body frame
}

void ReactiveOANode::computeRepulsiveVector() {
    std::vector<Eigen::Vector3f> distinct_forces;
    distinct_forces.reserve(SECTOR_NUM);
    
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(const auto& contour : obstacle.getContours(index)) {
            const auto& points = contour.getContour();
            if(points.size() < 2) continue;

            for(size_t i = 0; i < points.size() - 1; ++i) {
                Eigen::Vector3f p1(points[i].x, points[i].y, 0);
                Eigen::Vector3f p2(points[i+1].x, points[i+1].y, 0);
                Eigen::Vector3f segment = p2 - p1;
                
                if(segment.squaredNorm() < 1e-6f) continue;

                float t = -p1.dot(segment) / segment.squaredNorm();
                t = std::clamp(t, 0.0f, 1.0f);

                Eigen::Vector3f closest_point = p1 + segment * t;
                float dist = closest_point.norm();

                // Calculate Raw Force for this specific segment
                if(dist < safe_distance && dist > 1e-3f) {
                    float factor = (safe_distance - dist); 
                    Eigen::Vector3f new_force = -closest_point.normalized() * factor;

                    // --- CORE CLUSTERING LOGIC ---
                    bool merged = false;
                    for(auto& existing_force : distinct_forces) {
                        float similarity = new_force.normalized().dot(existing_force.normalized());

                        if(similarity > 0.9f) { // About 25 degree, keep max amplitude
                            if(new_force.squaredNorm() > existing_force.squaredNorm()) existing_force = new_force;
                            merged = true;
                            break; 
                        }
                    }

                    // New unique direction, add it
                    if(!merged) {
                        distinct_forces.push_back(new_force);
                    }
                }
            }
        }
    }

    repulsive_vec.setZero();
    for(const auto& f : distinct_forces) {
        repulsive_vec += f;
    }
    repulsive_vec = repulsive_vec.normalized();

    float min_dist = obstacle.getMinDistance();
    if(min_dist > safe_distance) min_dist = safe_distance;
    if(min_dist <= SELF_RADIUS) {
        RCLCPP_WARN(this->get_logger(), RED "⚠️ OBSTACLE ARE TOO CLOSE! ⚠️" RESET);
    }
    float urgency = (safe_distance - min_dist) / (safe_distance - HAZARD_DISTANCE + 0.01f); // Cut depth urgency
    urgency = std::clamp(urgency, 0.01f, 2.85f);

    const float relative_speed = fabs(movement_vec.dot(repulsive_vec));
    const float movement_bias = 0.75f;
    const float repulsive_force = urgency * (movement_bias*relative_speed + (1.0f - movement_bias)*SPEED_MAX_FORWARD);
    
    repulsive_vec = repulsive_vec * repulsive_force;
    RCLCPP_INFO(this->get_logger(), "%s Urgecy = %.1f%%, Repulsive = %.2f" RESET, urgency <= 1.0f ? YELLOW : PINK, urgency * 100, repulsive_vec.norm());
}

void ReactiveOANode::computeCorrectionVector() {
    correction_vec = control_vec + repulsive_vec;

    // Ensure correction vector is at most perpendicular to repulsive vector
    if (!repulsive_vec.isZero(1e-3f)) {
        Eigen::Vector3f repulsive_dir = repulsive_vec.normalized();
        const double dot = repulsive_dir.dot(correction_vec);

        const double bounce_bias = 0.01; // Tune-able
        if (dot < bounce_bias){
            correction_vec -= (dot - bounce_bias) * repulsive_dir;
        }
    }

    // Clamp correction vector to max speed
    correction_vec = Eigen::Vector3f(
        std::clamp(correction_vec.x(), -SPEED_MAX_FORWARD, SPEED_MAX_FORWARD),
        std::clamp(correction_vec.y(), -SPEED_MAX_STRAFE, SPEED_MAX_STRAFE),
        0 // std::clamp(correction_vec.x(), -SPEED_MAX_UP, SPEED_MAX_UP) // Still in 2D
    );

    correction_angular_vec = control_angular_vec;
}

void ReactiveOANode::resetVectors(){
    // Damping repulsive vector
    if(obstacle_clear_damping_counter == 0) {
        repulsive_vec.setZero();
    }
    else {
        repulsive_vec = repulsive_vec * REPULSIVE_DAMPING_CONSTANT;
    }

    // Reset the rest of vectors
    if(lost_control_signal) {
        control_vec.setZero();
        control_angular_vec.setZero();
    }

    movement_vec.setZero();
    movement_angular_vec.setZero();

    correction_vec.setZero();
    correction_angular_vec.setZero();
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

/*################################################# Node Loop */

void ReactiveOANode::nodeLoopCallback() {
    computeCorrectionVector();
    auto msg = ros2_msgs::msg::ControlInterface();

    msg.forward = correction_vec.x() / SPEED_MAX_FORWARD;
    msg.left = correction_vec.y() / SPEED_MAX_STRAFE;
    // msg.up = correction_vec.z() / SPEED_MAX_UP_FW; // Still in 2D

    if(lost_control_signal_counter < MISSED_FAST_TOPIC_THRESHOLD) lost_control_signal_counter++;
    if(lost_control_signal_counter >= MISSED_FAST_TOPIC_THRESHOLD) {
        if(lost_control_signal == false) RCLCPP_INFO(this->get_logger(), YELLOW "Waiting for control signal." RESET);
        lost_control_signal = true;
        last_control_signal = nullptr;
    }
    else {
        if(lost_control_signal == true) RCLCPP_INFO(this->get_logger(), GREEN "Received control signal." RESET);
    }

    if(!lost_control_signal) {
        msg.control_state = last_control_signal->control_state;
        msg.roll = last_control_signal->roll;
        msg.pitch = last_control_signal->pitch;
        msg.yaw = last_control_signal->yaw;
        msg.up = last_control_signal->up; // Still in 2D
        msg.control_by = last_control_signal->control_by;
        msg.wings_mode = last_control_signal->wings_mode;
    }
    else {
        RCLCPP_INFO(this->get_logger(), RED "Lost control signal!" RESET);
        msg.control_state = ros2_msgs::msg::ControlInterface::ARM;
        msg.control_by = ros2_msgs::msg::ControlInterface::REACTIVE_OA;
    }
    
    if(obstacle_clear_damping_counter) {
        msg.control_by = ros2_msgs::msg::ControlInterface::REACTIVE_OA;
        obstacle_clear_damping_counter--;
    }

    msg.header.stamp = this->get_clock()->now();
    #if PUBLISH_CORRECTION_CONTROL
    final_control_PUB->publish(msg);
    #endif

    #ifdef VISUALIZE
    publishVectorArrow(control_vec_PUB, control_vec, 0.0f, 0.0f, 1.0f); // Blue
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0f, 0.7f, 0.7f); // Teal
    publishVectorArrow(repulsive_vec_PUB, repulsive_vec, 0.75f, 0.75f, 0.0f); // Yellow
    publishVectorArrow(correction_vec_PUB, correction_vec, 0.0f, 1.0f, 0.0f); // Green
    #endif

    resetVectors();
}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const ros2_msgs::msg::ControlInterface::SharedPtr msg){
    lost_control_signal = false;
    lost_control_signal_counter = 0;

    last_control_signal = msg;
    computeControlVector();
}

void ReactiveOANode::closeContourCallback(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_clear_damping_counter = OBSTACLE_DAMPING_INIT;

    obstacle.topicToObstacle(msg);
    safe_distance = obstacle.safe_distance;
    computeRepulsiveVector();
}

void ReactiveOANode::odoCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    last_odo = msg;
    computeMovementVector();
}
