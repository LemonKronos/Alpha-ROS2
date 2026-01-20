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

    perception_SUB = this->create_subscription<ros2_msgs::msg::FusePerception>(
        FUSE_PERCEPTION_TOPIC,
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
    // We will store distinct forces here instead of summing immediately
    std::vector<Eigen::Vector3f> distinct_forces;
    
    // Iterate through all 12 Sectors
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(const auto& contour : obstacle.getContours(index)) {
            const auto& points = contour.getContour();
            if(points.size() < 2) continue;

            for(size_t i = 0; i < points.size() - 1; ++i) {
                // ... [Same Segment Calculation Logic as before] ...
                Eigen::Vector3f p1(points[i].x, points[i].y, 0);
                Eigen::Vector3f p2(points[i+1].x, points[i+1].y, 0);
                Eigen::Vector3f segment = p2 - p1;
                
                if(segment.squaredNorm() < 1e-6f) continue;

                float t = -p1.dot(segment) / segment.squaredNorm();
                if (t < 0.0f) t = 0.0f;
                else if (t > 1.0f) t = 1.0f;

                Eigen::Vector3f closest_point = p1 + segment * t;
                float dist = closest_point.norm();

                // Calculate Raw Force for this specific segment
                if(dist < safe_distance && dist > 1e-3f) {
                    float factor = (safe_distance - dist); 
                    Eigen::Vector3f new_force = -closest_point.normalized() * factor;

                    // --- CLUSTERING LOGIC STARTS HERE ---
                    bool merged = false;
                    for(auto& existing_force : distinct_forces) {
                        // Check alignment using Dot Product
                        // If normalized dot > 0.866, they are within ~30 degrees
                        // We use unnormalized dot for speed, knowing directions are similar
                        // Let's use cosine similarity safely:
                        
                        float similarity = new_force.normalized().dot(existing_force.normalized());

                        // Threshold: 0.9 is approx 25 degrees cone. 
                        // If they push in same direction, treat as same obstacle source.
                        if(similarity > 0.9f) { 
                            // Take the MAX magnitude (Keep the one that pushes harder)
                            if(new_force.squaredNorm() > existing_force.squaredNorm()) {
                                existing_force = new_force;
                            }
                            merged = true;
                            break; 
                        }
                    }

                    // If it's a unique direction (e.g. a different wall), add it
                    if(!merged) {
                        distinct_forces.push_back(new_force);
                    }
                    // --- CLUSTERING LOGIC ENDS HERE ---
                }
            }
        }
    }

    // Now simply sum the distinct forces
    repulsive_vec = Eigen::Vector3f::Zero();
    for(const auto& f : distinct_forces) {
        repulsive_vec += f;
    }

    // Snap to 0
    if(repulsive_vec.isZero(1e-4f)) {
        repulsive_vec.setZero();
        return;
    }

    // Scale repulsive vector (Your original scaling logic)
    // Note: You might need to re-tune 'scale' slightly since we aren't summing as many vectors now.
    float min_dist = obstacle.getMinDistance();
    if(min_dist > safe_distance) min_dist = safe_distance;
    
    const float x = (safe_distance - min_dist) / safe_distance;
    const float scale = 2.0f * x; 
    
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
    #if PUBLISH_CORRECTION_CONTROL
    final_control_PUB->publish(msg);
    #endif
    
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
