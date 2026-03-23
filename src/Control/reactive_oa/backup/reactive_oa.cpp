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
        std::bind(&ReactiveOANode::inputControlCallback, this, _1));

    close_contour_SUB = this->create_subscription<alpha_msgs::msg::Lidar2dObstacle>(
        Topic::LIDAR_2D_CONTOUR_CLOSE,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::closeContourCallback, this, _1));

    perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
        Topic::FUSE_PERCEPTION,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::perceptionCallback, this, _1));

    // Timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );

    // Init variables
    control_vec.setZero();
    movement_vec.setZero();
    repulsive_vec.setZero();
    correction_vec.setZero();
    // control_angular_vec.setZero();
    // movement_angular_vec.setZero();
    // repulsive_angular_vec.setZero();
    // correction_angular_vec.setZero();

    repulsive_damping_counter = 0;
    obstacle_clear_damping_counter = 0;

    safe_distance = Drone::HAZARD_DISTANCE;

    lost_control_signal = true;
    lost_control_signal_counter = Threshold::MISSED_FAST_TOPIC;
    last_control_signal = nullptr;

    lost_perception = true;
    lost_perception_counter = Threshold::MISSED_FAST_TOPIC;
    last_perception = nullptr;

    obstacle_rate_mismatch_counter = 0;
}
ReactiveOANode::~ReactiveOANode(){}

/*################################################# Methods*/

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
    
    const float approach_angle = fabs(movement_vec.normalized().dot(repulsive_vec));
    std::clamp(approach_angle, 0.01f, 1.0f);

    float min_dist = obstacle.getMinDistance();
    if(min_dist > safe_distance) min_dist = safe_distance;
    if(min_dist <= Drone::RADIUS) {
        RCLCPP_WARN(this->get_logger(), RED "⚠️ OBSTACLE ARE TOO CLOSE! ⚠️" RESET);
    }

    float urgency = (safe_distance - min_dist) / (safe_distance - Drone::HAZARD_DISTANCE + 0.01f); // Cut depth urgency
    urgency = std::clamp(urgency, 0.01f, 1.0f);

    const float repulsive_force = control_vec.norm() + urgency * Drone::SPEED_MAX_FORWARD;

    repulsive_vec = repulsive_vec * repulsive_force * approach_angle;

    RCLCPP_INFO(this->get_logger(), BLUE "Repulsive vec = %0.2f" RESET, repulsive_vec.norm());
    // RCLCPP_INFO(this->get_logger(), "%s Urgecy = %.1f%%, Repulsive = %.2f" RESET, urgency <= 1.0f ? YELLOW : PINK, urgency * 100, repulsive_vec.norm());
}

void ReactiveOANode::computeCorrectionVector() {
    // 1. Gather all active constraints (obstacle points within a threshold)
    std::vector<Eigen::Vector3f> hazards;
    for(uint8_t index = 0; index < SECTOR_NUM; index++) {
        for(const auto& contour : obstacle.getContours(index)) {
            for(const auto& pt : contour.getContour()) {
                Eigen::Vector3f p(pt.x, pt.y, 0);
                float dist = p.norm();
                if(dist > 0.1f && dist < safe_distance) { 
                    hazards.push_back(p);
                }
            }
        }
    }

    int num_hazards = hazards.size();
    int num_vars = 3; // u_x, u_y, u_z
    int num_constraints = num_hazards + 3; // hazards + 3 kinematic speed limits

    // 2. Setup Objective Matrix P and Vector q
    // Minimize: 0.5 * u^T * P * u + q^T * u
    // To minimize ||u - control_vec||^2, P = 2*I, q = -2*control_vec
    Eigen::SparseMatrix<double> P(num_vars, num_vars);
    P.insert(0,0) = 3.0;
    P.insert(1,1) = 2.0;
    P.insert(2,2) = 5.0;

    Eigen::VectorXd q(num_vars);
    q << -2.0 * control_vec.x(), -2.0 * control_vec.y(), -2.0 * control_vec.z();

    // 3. Setup Constraint Matrix A, and Bounds (lower_bound <= A*u <= upper_bound)
    Eigen::SparseMatrix<double> A(num_constraints, num_vars);
    Eigen::VectorXd lower_bound(num_constraints);
    Eigen::VectorXd upper_bound(num_constraints);

    // Populate Matrix A with triplets for efficiency
    std::vector<Eigen::Triplet<double>> A_triplets;
    
    float gamma = 1.0f; // Tuning parameter: how aggressively to brake near the wall

    for(int i = 0; i < num_hazards; ++i) {
        Eigen::Vector3f h = hazards[i];
        float dist = h.norm();
        Eigen::Vector3f dir = h.normalized();

        // A matrix row: the direction vector to the hazard
        A_triplets.push_back(Eigen::Triplet<double>(i, 0, dir.x()));
        A_triplets.push_back(Eigen::Triplet<double>(i, 1, dir.y()));
        A_triplets.push_back(Eigen::Triplet<double>(i, 2, dir.z()));

        // We only care about upper bound (max speed towards wall). Lower bound is negative infinity.
        lower_bound(i) = -OsqpEigen::INFTY; 
        
        // CBF Rule: velocity towards wall <= gamma * (distance - safe_distance)
        float max_safe_approach_speed = gamma * (dist - Drone::HAZARD_DISTANCE);
        upper_bound(i) = static_cast<double>(max_safe_approach_speed);
    }

    // Add Kinematic Speed Limits to the QP so it doesn't solve for impossible speeds
    // u_x limit
    A_triplets.push_back(Eigen::Triplet<double>(num_hazards, 0, 1.0));
    lower_bound(num_hazards) = -Drone::SPEED_MAX_FORWARD;
    upper_bound(num_hazards) = Drone::SPEED_MAX_FORWARD;

    // u_y limit
    A_triplets.push_back(Eigen::Triplet<double>(num_hazards + 1, 1, 1.0));
    lower_bound(num_hazards + 1) = -Drone::SPEED_MAX_STRAFE;
    upper_bound(num_hazards + 1) = Drone::SPEED_MAX_STRAFE;

    // u_z limit
    A_triplets.push_back(Eigen::Triplet<double>(num_hazards + 2, 2, 1.0));
    lower_bound(num_hazards + 2) = -Drone::SPEED_MAX_UP;
    upper_bound(num_hazards + 2) = Drone::SPEED_MAX_UP;

    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // 4. Feed the solver
    // Because the number of constraints changes dynamically, we must completely clear 
    // the workspace and the matrix data, rather than trying to update them in place.
    solver.clearSolver();
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();
    
    solver.data()->setNumberOfVariables(num_vars);
    solver.data()->setNumberOfConstraints(num_constraints);
    
    if(!solver.data()->setHessianMatrix(P)) return;
    if(!solver.data()->setGradient(q)) return;
    if(!solver.data()->setLinearConstraintsMatrix(A)) return;
    if(!solver.data()->setLowerBound(lower_bound)) return;
    if(!solver.data()->setUpperBound(upper_bound)) return;

    // SHUT OSQP UP
    solver.settings()->setVerbosity(false);

    // Initialize the fresh solver
    if(!solver.initSolver()) return;

    // 5. Extract the safe correction vector
    if(solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        Eigen::VectorXd QPSolution = solver.getSolution();
        correction_vec = Eigen::Vector3f(QPSolution(0), QPSolution(1), QPSolution(2));
    } else {
        // If the solver fails (usually means completely trapped), default to a hard hover
        RCLCPP_WARN(this->get_logger(), RED "CBF Solver Failed! Forcing Hover" RESET);
        correction_vec.setZero();
    }
}

void ReactiveOANode::resetVectors() {
    // Reset control signal
    if(lost_control_signal) {
        control_vec.setZero();
    }

    // Mismatch topic reset
    if(obstacle_clear_damping_counter == 0) {
        repulsive_vec.setZero();
    }

    // Leave correctin vec for damping
    // correction_vec.setZero();

    // Reset movement
    if(lost_perception) {
        movement_vec.setZero();
    }
}

#ifdef VISUALIZE
void ReactiveOANode::publishVectorArrow(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
    const Eigen::Vector3f& vec,
    float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "alpha_minus_2_0/base_link";
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

/*################################################# Node Loop */

void ReactiveOANode::nodeLoopCallback() {
    computeCorrectionVector();

    auto msg = alpha_msgs::msg::ControlInterface();

    msg.forward = correction_vec.x() / Drone::SPEED_MAX_FORWARD;
    msg.left = correction_vec.y() / Drone::SPEED_MAX_STRAFE;
    msg.up = correction_vec.z() / Drone::SPEED_MAX_UP;

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
        msg.up = last_control_signal->up;
        msg.control_by = last_control_signal->control_by;
        msg.wings_mode = last_control_signal->wings_mode;
    }
    else {
        msg.control_state = alpha_msgs::msg::ControlInterface::ARM;
        msg.control_by = alpha_msgs::msg::ControlInterface::REACTIVE_OA;
    }
    
    // Obstacle callback check
    if(obstacle_rate_mismatch_counter < Threshold::MISMATCH_RATE_TOPIC) obstacle_rate_mismatch_counter++;
    else {
        // Obstacle damping
        if(obstacle_clear_damping_counter) {
            msg.control_by = alpha_msgs::msg::ControlInterface::REACTIVE_OA;
            obstacle_clear_damping_counter--;
        }
    }

    // Publish msg
    msg.header.stamp = this->get_clock()->now();
    #if PUBLISH_CORRECTION_CONTROL
    final_control_PUB->publish(msg);
    #endif

    #ifdef VISUALIZE
    publishVectorArrow(control_vec_PUB, control_vec, 0.0f, 0.0f, 1.0f); // Blue
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0f, 0.7f, 0.7f); // Teal
    publishVectorArrow(repulsive_vec_PUB, repulsive_vec, 1.0f, 0.1f, 0.7f); // Pink
    publishVectorArrow(correction_vec_PUB, correction_vec, 1.0f, 0.0f, 0.0f); // Red
    #endif

    resetVectors();
}

/*################################################# Callbacks*/

void ReactiveOANode::inputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg){
    lost_control_signal = false;
    lost_control_signal_counter = 0;

    last_control_signal = msg;
    computeControlVector();
}

void ReactiveOANode::closeContourCallback(const alpha_msgs::msg::Lidar2dObstacle::SharedPtr msg){
    obstacle_clear_damping_counter = OBSTACLE_DAMPING_INIT;
    obstacle_rate_mismatch_counter = 0;

    obstacle.topicToObstacle(msg);
    safe_distance = obstacle.safe_distance;
    computeRepulsiveVector();
}

void ReactiveOANode::perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg){
    lost_perception_counter = 0;

    last_perception = msg;
    computeMovementVector();
}

void ReactiveOANode::initCBFSolver() {
    solver.settings()->setVerbosity(false); // Keep the terminal clean
    solver.settings()->setWarmStart(true);  // Speeds up consecutive solves
    solver.settings()->setMaxIteration(4000);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
}