#include "reactive_oa/reactive_oa.hpp"

ReactiveOANode::ReactiveOANode(): Node("reactive_oa_node"){
    
    Global::setup_for_simulation(this);
    
    // Publisher
    final_control_PUB = this->create_publisher<alpha_msgs::msg::ControlInterface>(Topic::CONTROL_REACTIVE, 10);

    #ifdef VISUALIZE
        control_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/control_vector", 10);
        movement_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/movement_vector", 10);
        repulsive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/repulsive_vector", 10);
        correction_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/correction_vector", 10);
    #endif

    // Subscriber
    input_control_SUB = this->create_subscription<alpha_msgs::msg::ControlInterface>(
        Topic::CONTROL_INPUT,
        10,
        std::bind(&ReactiveOANode::inputControlCallback, this, _1)
    );

    seeing_VFH_SUB = this->create_subscription<alpha_msgs::msg::VectorFieldHistogram>(
        Topic::VOXEL_HAZARD_SEEING,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::seeingVoxelCallback, this, _1)
    );

    perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
        Topic::FUSE_PERCEPTION,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::perceptionCallback, this, _1)
    );

    // Timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );

    // Check sycn
    alpha_msgs::msg::VectorFieldHistogram test_msg;
    if(test_msg.vfh_part.size() != Sensor::VFH_MSG_CHUNK_SIZE) {
        RCLCPP_ERROR(this->get_logger(), RED "Wrong VFH msg size, please update to %d" RESET, Sensor::VFH_MSG_CHUNK_SIZE);
        return;
    }

    // Init variables
    control_vec.setZero();
    movement_vec.setZero();
    repulsive_vec.setZero();
    correction_vec.setZero();
    // control_angular_vec.setZero();
    // movement_angular_vec.setZero();
    // repulsive_angular_vec.setZero();
    // correction_angular_vec.setZero();

    VFH.reset();

    lost_control_signal = true;
    lost_control_signal_counter = Threshold::MISSED_FAST_TOPIC;
    last_control_signal = nullptr;

    lost_perception = true;
    lost_perception_counter = Threshold::MISSED_FAST_TOPIC;
    last_perception = nullptr;

    hazard_distance = Drone::HAZARD_DISTANCE;
    has_seeing_voxel_counter = HAS_SEEING_VOXEL_COUNTER_INIT;
}
ReactiveOANode::~ReactiveOANode(){}

#pragma region Compute vectors

void ReactiveOANode::computeControlVector() {
    control_vec = Eigen::Vector3f(
        last_control_signal->forward * Drone::SPEED_MAX_FORWARD,
        last_control_signal->left * Drone::SPEED_MAX_FORWARD,
        last_control_signal->up * Drone::SPEED_MAX_UP
    ); // body frame

    // RCLCPP_INFO(this->get_logger(), BLUE "Control vec = %0.2f" RESET, control_vec.norm());

    // control_angular_vec = Eigen::Vector3f(
    //     last_control_signal->roll,
    //     last_control_signal->pitch,
    //     last_control_signal->yaw
    // ); // body frame
}

void ReactiveOANode::computeMovementVector() {
    movement_vec = Eigen::Vector3f(
        last_perception->velocity[0],
        last_perception->velocity[1],
        last_perception->velocity[2]
    ); // world frame
    if(movement_vec.hasNaN()) {
        movement_vec.setZero();
        // RCLCPP_WARN(this->get_logger(), RED "Movement has NaN" RESET);
    }
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_perception->q);
    movement_vec = q.inverse() * movement_vec; // body frame

    // movement_angular_vec = Eigen::Vector3f(
    //     last_perception->angular_velocity[0],
    //     last_perception->angular_velocity[1],
    //     last_perception->angular_velocity[2]
    // ); // world frame
    // movement_angular_vec = q.inverse() * movement_angular_vec; // body frame
}

void ReactiveOANode::computeVectorFieldHistogram(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg) {
    VFH.reset();

    // Extract the VFH
    for(size_t i = 0; i < Sensor::VFH_TOTAL_BINS; i++) {
        VFH[i] = (msg->vfh_part[i / Sensor::VFH_MSG_BIT_SIZE] >> (i % Sensor::VFH_MSG_BIT_SIZE)) & 1;
    }

    // Get the closest point repulsion
    computeRepulsiveVector({msg->closest_obstacle.x, msg->closest_obstacle.y, msg->closest_obstacle.z});
}

void ReactiveOANode::computeRepulsiveVector(const Eigen::Vector3f point) {
    float strenght = (hazard_distance - point.z()) / (hazard_distance - Drone::HAZARD_DISTANCE + 0.01f); // Linear cut depth repulsive strength
    strenght *= Drone::SPEED_MAX_FORWARD;
    repulsive_vec = -math_utils::toCartesian({point.x(), point.y(), strenght});
}

void ReactiveOANode::computeCorrectionVector() {
    Eigen::Vector3f avoidance_vec = repulsive_vec + control_vec;
    Eigen::Vector3f target_vec = control_vec.squaredNorm() > avoidance_vec.squaredNorm() ? control_vec : avoidance_vec;
    float target_speed = target_vec.norm();
    if(target_speed < 1e-3f) {
        correction_vec.setZero();
        return;
    }

#if DEBUG
    Eigen::Vector3f spherical_target_vec = math_utils::toSpherical(target_vec);
    RCLCPP_INFO(
        this->get_logger(), GREEN "Target = %.2f, yaw = %.0f, pitch = %.0f" RESET, 
        spherical_target_vec.z(), 
        spherical_target_vec.x() / DEGREE, 
        spherical_target_vec.y() / DEGREE
    );
#endif

    Eigen::Vector3f target_direction = target_vec.normalized();
    Eigen::Vector3f movement_direction = movement_vec.squaredNorm() > 1e-6f ?  movement_vec.normalized() : target_direction;

    constexpr float CONTROL_WEIGHT = 1.0f;
    constexpr float MOVEMENT_WEIGHT = 0.3f;

    float min_cost = std::numeric_limits<float>::max();
    Eigen::Vector3f best_direction = Eigen::Vector3f::Zero();
    bool found_safe_path = false;

    for(int i = 0; i < Sensor::VFH_TOTAL_BINS; i++) {
        if(VFH[i]) continue; // Blocked path

        int row = i / Sensor::VFH_AZIMUTH_BINS;
        int col = i % Sensor::VFH_AZIMUTH_BINS;

        float yaw = ((col * Sensor::VFH_RESOLUTION) - M_PI + Sensor::VFH_RESOLUTION / 2);
        float pitch = ((row * Sensor::VFH_RESOLUTION) - M_PI_2 + Sensor::VFH_RESOLUTION / 2);

        Eigen::Vector3f canditate_direction = math_utils::toCartesian({yaw, pitch, 1.0f});

        float target_cost = 1.0f - canditate_direction.dot(target_direction);
        float movement_cost = 1.0f - canditate_direction.dot(movement_direction);

        float total_cost = CONTROL_WEIGHT * target_cost + MOVEMENT_WEIGHT * movement_cost;

        if(total_cost < min_cost) {
            min_cost = total_cost;
            best_direction = canditate_direction;
            found_safe_path = true;
        }
    }

    if(!found_safe_path) {
        correction_vec.setZero();
        return;
    }

    float deviation_factor = std::max(0.0f, best_direction.dot(target_direction));

    float safe_speed = target_speed * deviation_factor;
    
    correction_vec = best_direction * safe_speed;

#if DEBUG
    Eigen::Vector3f spherical_correction_vec = math_utils::toSpherical(correction_vec);
    RCLCPP_INFO(
        this->get_logger(), 
        YELLOW "Correction = %.2f, yaw = %.0f, pitch = %.0f" RESET, 
        spherical_correction_vec.z(), 
        spherical_correction_vec.x() / DEGREE, 
        spherical_correction_vec.y() / DEGREE
    );
#endif
}

void ReactiveOANode::resetVectors() {
    // Reset control signal
    if(lost_control_signal) {
        control_vec.setZero();
    }

    if(!has_seeing_voxel_counter) {
        VFH.reset();
        repulsive_vec.setZero();
    }
    // Reset correction vector
    correction_vec.setZero();

    // Reset movement
    if(lost_perception) {
        movement_vec.setZero();
    }
}

#pragma endregion

#ifdef VISUALIZE
void ReactiveOANode::publishVectorArrow(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const Eigen::Vector3f& vec,
    float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = this->base_link.get();
    arrow.header.stamp = this->now();
    arrow.ns = "oa_vectors";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // Arrow scale (bigger arrow)
    arrow.scale.x = 0.2;  // shaft thickness
    arrow.scale.y = 0.2;   // head diameter
    arrow.scale.z = 0.2;   // head length

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

void ReactiveOANode::nodeLoopCallback() {
    computeCorrectionVector();

    auto msg = alpha_msgs::msg::ControlInterface();

    #if DO_REACTIVE_OA
    msg.forward = correction_vec.x() / Drone::SPEED_MAX_FORWARD;
    msg.left = correction_vec.y() / Drone::SPEED_MAX_STRAFE;
    msg.up = correction_vec.z() / Drone::SPEED_MAX_UP;
    #else
    msg.forward  = last_control_signal->forward;
    msg.left = last_control_signal->left;
    msg.up = last_control_signal->up;
    #endif

    // Odometry callback check
    if(lost_perception_counter < Threshold::MISSED_FAST_TOPIC) lost_perception_counter++;
    if(lost_perception_counter >= Threshold::MISSED_FAST_TOPIC) {
        if(lost_perception == false) RCLCPP_INFO(this->get_logger(), YELLOW "Lost odometry" RESET);
        lost_perception = true;
        last_perception = nullptr;
    }
    else {
        if(lost_perception == true) RCLCPP_INFO(this->get_logger(), GREEN "Received odometry" RESET);
        lost_perception = false;
    }

    // Control signal callback check
    if(lost_control_signal_counter < Threshold::MISSED_FAST_TOPIC) lost_control_signal_counter++;
    if(lost_control_signal_counter >= Threshold::MISSED_FAST_TOPIC) {
        if(lost_control_signal == false) RCLCPP_INFO(this->get_logger(), YELLOW "Waiting for control signal." RESET);
        lost_control_signal = true;
        last_control_signal = nullptr;
    }
    else {
        if(lost_control_signal == true) RCLCPP_INFO(this->get_logger(), GREEN "Received control signal." RESET);
        lost_control_signal = false;
    }

    // Build msg
    if(!lost_control_signal) {
        msg.control_state = last_control_signal->control_state;

        msg.roll = last_control_signal->roll;
        msg.pitch = last_control_signal->pitch;
        msg.yaw = last_control_signal->yaw;
        
        msg.control_by = last_control_signal->control_by;
        msg.wings_mode = last_control_signal->wings_mode;
    }
    else {
        msg.control_state = alpha_msgs::msg::ControlInterface::ARM;
        msg.control_by = alpha_msgs::msg::ControlInterface::REACTIVE_OA;
    }

    // Check seeing hazard voxel callback
    if(has_seeing_voxel_counter > 0) has_seeing_voxel_counter--;

    // Publish msg
    msg.header.stamp = this->get_clock()->now();
    final_control_PUB->publish(msg);

    #ifdef VISUALIZE
    publishVectorArrow(control_vec_PUB, control_vec, 0.0f, 1.0f, 0.0f); // Green
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0f, 0.0f, 1.0f); // Blue
    publishVectorArrow(correction_vec_PUB, correction_vec, 1.0f, 1.0f, 0.0f); // Yellow 
    #endif

    resetVectors();
}

#pragma region Callbacks 

void ReactiveOANode::inputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg){
    lost_control_signal = false;
    lost_control_signal_counter = 0;

    last_control_signal = msg;
    computeControlVector();
}

void ReactiveOANode::seeingVoxelCallback(const alpha_msgs::msg::VectorFieldHistogram::SharedPtr msg) {
    has_seeing_voxel_counter = HAS_SEEING_VOXEL_COUNTER_INIT;
    computeVectorFieldHistogram(msg);
}

void ReactiveOANode::perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg){
    lost_perception_counter = 0;

    last_perception = msg;
    computeMovementVector();
}

#pragma endregion 
