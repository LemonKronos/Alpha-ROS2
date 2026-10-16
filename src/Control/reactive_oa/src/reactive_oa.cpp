#include "reactive_oa/reactive_oa.hpp"

spatial_container::ReactiveOANode::ReactiveOANode(const rclcpp::NodeOptions &options) :
Node("reactive_oa_node", options)
{
    Global::setup_for_simulation(this);
    
    // Publishers
    final_control_PUB = this->create_publisher<alpha_msgs::msg::ControlInterface>(Topic::CONTROL_REACTIVE, rclcpp::SensorDataQoS());
    spatial_obs_PUB = this->create_publisher<alpha_msgs::msg::SpatialObservation>("/spatial_container/spatial_observation", rclcpp::SensorDataQoS());

    #if VISUALIZE
        control_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/control_vector", 10);
        movement_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/movement_vector", 10);
        repulsive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/repulsive_vector", 10);
        correction_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/correction_vector", 10);
        foam_spheres_PUB = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualizer/foam_spheres", 10);
    #endif

    // Subscribers
    input_control_SUB = this->create_subscription<alpha_msgs::msg::ControlInterface>(
        Topic::CONTROL_INPUT,
        rclcpp::SensorDataQoS(),
        std::bind(&ReactiveOANode::inputControlCallback, this, _1)
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

    // Initialize Vectors
    control_vec.setZero();
    movement_vec.setZero();
    total_repulsive_vec.setZero();
    correction_vec.setZero();

    lost_control_signal = true;
    lost_control_signal_counter = Threshold::ALLOW_MISSED_FAST_TOPIC;
    lost_perception = true;
    lost_perception_counter = Threshold::ALLOW_MISSED_FAST_TOPIC;

    // Hardcode dummy FOAM skeleton (Center body + 4 arms). TODO: load from global YAML
    foam_spheres.push_back(Eigen::Vector4f(0.0f, 0.0f, 0.0f, 0.3f));     // Core
    foam_spheres.push_back(Eigen::Vector4f(0.2f, 0.2f, 0.05f, 0.15f));   // Front-Left
    foam_spheres.push_back(Eigen::Vector4f(0.2f, -0.2f, 0.05f, 0.15f));  // Front-Right
    foam_spheres.push_back(Eigen::Vector4f(-0.2f, 0.2f, 0.05f, 0.15f));  // Back-Left
    foam_spheres.push_back(Eigen::Vector4f(-0.2f, -0.2f, 0.05f, 0.15f)); // Back-Right

    #if TIME_ANALYSE
    analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger(), 800);
    #endif
}

spatial_container::ReactiveOANode::~ReactiveOANode(){
#if TIME_ANALYSE
    analyzer->printSummary();
#endif
}

void spatial_container::ReactiveOANode::computeControlVector() {
    control_vec = Eigen::Vector3f(
        last_control_signal->forward * Drone::SPEED_MAX_FORWARD,
        last_control_signal->left * Drone::SPEED_MAX_STRAFE,
        last_control_signal->up * Drone::SPEED_MAX_UP
    ); 
}

void spatial_container::ReactiveOANode::computeMovementVector() {
    Eigen::Vector3f world_vel(
        last_perception->velocity[0],
        last_perception->velocity[1],
        last_perception->velocity[2]
    );
    if(world_vel.hasNaN()) world_vel.setZero();
    
    Eigen::Quaternionf q = frame_utils::arrayToQuaternion(last_perception->q);
    movement_vec = q.inverse() * world_vel; 
}

void spatial_container::ReactiveOANode::extractAndPublishSpatialBoxes() {
#if TIME_ANALYSE
    analyzer->start_segment("Extract Spatial Boxes");
#endif

    auto obs_msg = alpha_msgs::msg::SpatialObservation();
    obs_msg.header.stamp = this->now();
    obs_msg.header.frame_id = base_link.get();

    Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);
    Eigen::Matrix3f body_rot = drone_q.toRotationMatrix();

    // --- Coarse Box Setup ---
    float coarse_stride = (10.0f * UNIT_TO_METER) / BOX_DIM;
    float coarse_ref_dist = std::sqrt(3.0f * std::pow(5.0f * UNIT_TO_METER, 2));

    // --- Detail Box Setup ---
    float intent_norm = std::clamp(control_vec.norm() / Drone::SPEED_MAX_FORWARD, 0.0f, 1.0f);
    float l_v = (5.0f + 15.0f * intent_norm) * UNIT_TO_METER;
    float detail_stride_x = l_v / BOX_DIM;
    float detail_stride_yz = (5.0f * UNIT_TO_METER) / BOX_DIM;
    float detail_ref_dist = std::sqrt(std::pow(l_v, 2) + 2.0f * std::pow(2.5f * UNIT_TO_METER, 2));

    //? Intent Rotation Matrix (Align X with control_vec, Z roughly up)
    Eigen::Matrix3f intent_rot = body_rot; 
    if (control_vec.norm() > 1e-3f) {
        Eigen::Vector3f intent_x = body_rot * control_vec.normalized();
        Eigen::Vector3f intent_y = Eigen::Vector3f::UnitZ().cross(intent_x).normalized();
        if(intent_y.hasNaN()) intent_y = Eigen::Vector3f::UnitY();
        Eigen::Vector3f intent_z = intent_x.cross(intent_y).normalized();
        intent_rot.col(0) = intent_x;
        intent_rot.col(1) = intent_y;
        intent_rot.col(2) = intent_z;
    }

    int coarse_idx = 0;
    int detail_idx = 0;

    // Loop Z, Y, X (Channel-First flattening)
    for (int z = 0; z < BOX_DIM; ++z) {
        for (int y = 0; y < BOX_DIM; ++y) {
            for (int x = 0; x < BOX_DIM; ++x) {
                
                // Coarse Cell (queries distance only)
                Eigen::Vector3f c_local((x - BOX_DIM/2.0f + 0.5f) * coarse_stride, (y - BOX_DIM/2.0f + 0.5f) * coarse_stride, (z - BOX_DIM/2.0f + 0.5f) * coarse_stride);
                Eigen::Vector3f c_world = drone_pos + body_rot * c_local;
                
                float c_dist = 0.0f;
                bool c_obs = spatial_container::global_map.getDistance(c_world, c_dist);

                obs_msg.coarse_box[coarse_idx] = c_obs ? (c_dist / coarse_ref_dist) : 1.0f;
                obs_msg.coarse_box[1728 + coarse_idx] = c_obs ? 1.0f : 0.0f;
                
                // Detail Cell (queries distance only)
                Eigen::Vector3f d_local((x + 0.5f) * detail_stride_x, (y - BOX_DIM/2.0f + 0.5f) * detail_stride_yz, (z - BOX_DIM/2.0f + 0.5f) * detail_stride_yz);
                Eigen::Vector3f d_world = drone_pos + intent_rot * d_local;

                float d_dist = 0.0f;
                bool d_obs = spatial_container::global_map.getDistance(d_world, d_dist);

                float d_val = d_obs ? (d_dist / detail_ref_dist) : 1.0f;
                obs_msg.detail_box[detail_idx] = d_val;         
                obs_msg.detail_box[1728 + detail_idx] = d_val;  
                obs_msg.detail_box[3456 + detail_idx] = d_obs ? 1.0f : 0.0f; 

                coarse_idx++;
                detail_idx++;
            }
        }
    }

    spatial_obs_PUB->publish(obs_msg);

    #if TIME_ANALYSE
        analyzer->stop_segment("Extract Spatial Boxes");
    #endif
}

void spatial_container::ReactiveOANode::computeMultiSphereGradient() {
    #if TIME_ANALYSE
        analyzer->start_segment("Compute Multi-Sphere Gradient");
    #endif
    
    total_repulsive_vec.setZero();
    Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);

    float speed_factor = std::clamp(movement_vec.norm() / Drone::SPEED_MAX_FORWARD, 0.0f, 1.0f);
    float speed_inflation = 1.0f + (1.5f * speed_factor);

    for (const auto& sphere : foam_spheres) {
        Eigen::Vector3f center_body = sphere.head<3>();
        float base_radius = sphere.w();
        float dynamic_radius = base_radius * speed_inflation;

        Eigen::Vector3f center_world = drone_pos + (drone_q * center_body);

        float distance = 0.0f;
        // First, check distance alone
        bool observed = spatial_container::global_map.getDistance(center_world, distance);

        if(!observed) RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this-> get_clock(), 1000,
            RED "Unable to get ESDF distance at (%.1f, %.1f, %.1f)" RESET,
            center_world.x(),
            center_world.y(),center_world.z()
        );

        // Only query the gradient if the sphere's safety margin is breached
        // if (observed && distance < dynamic_radius) {
            Eigen::Vector3f gradient = Eigen::Vector3f::Zero();
            if(!spatial_container::global_map.getGradient(center_world, gradient)) RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this-> get_clock(), 1000,
                RED "Unable to get ESDF gradient at (%.1f, %.1f, %.1f)" RESET,
                center_world.x(),
                center_world.y(),center_world.z()
            );

            float urgency = (dynamic_radius - distance) / dynamic_radius;
            urgency = std::clamp(urgency, 0.0f, 1.0f);
            
            Eigen::Vector3f push_world = gradient.normalized() * urgency * Drone::SPEED_MAX_FORWARD;
            Eigen::Vector3f push_body = drone_q.inverse() * push_world;

            total_repulsive_vec += push_body;
        // }
    }

    #if TIME_ANALYSE
        analyzer->stop_segment("Compute Multi-Sphere Gradient");
    #endif
}

void spatial_container::ReactiveOANode::computeCorrectionVector() {
    Eigen::Vector3f target_vec = control_vec + total_repulsive_vec;
    
    // Hard cutoff if opposing vectors cancel out perfectly near an obstacle
    if(target_vec.norm() < 1e-3f) {
        correction_vec.setZero();
    } else {
        // Here you would add smoothing or kinematic limits if necessary. 
        // For now, it passes the pure summed potential field vector.
        correction_vec = target_vec;
    }
}

void spatial_container::ReactiveOANode::resetVectors() {
    if(lost_control_signal) control_vec.setZero();
    if(lost_perception) movement_vec.setZero();
    correction_vec.setZero();
    total_repulsive_vec.setZero();
}

void spatial_container::ReactiveOANode::nodeLoopCallback() {
#if TIME_ANALYSE
    analyzer->start_segment("Main Loop");
#endif

    // Update statuses
    if(lost_perception_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) lost_perception_counter++;
    lost_perception = (lost_perception_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC);

    if(lost_control_signal_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) lost_control_signal_counter++;
    lost_control_signal = (lost_control_signal_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC);

    if (!lost_perception && !lost_control_signal) {
        // extractAndPublishSpatialBoxes();
        computeMultiSphereGradient();
    }

    computeCorrectionVector();

    // Publish Final Control
    auto msg = alpha_msgs::msg::ControlInterface();
    msg.header.stamp = this->now();
    
    if(!lost_control_signal) {
        #if DO_REACTIVE_OA
        msg.forward = correction_vec.x() / Drone::SPEED_MAX_FORWARD;
        msg.left = correction_vec.y() / Drone::SPEED_MAX_STRAFE;
        msg.up = correction_vec.z() / Drone::SPEED_MAX_UP;
        #else
        msg.forward = last_control_signal->forward;
        msg.left = last_control_signal->left;
        msg.up = last_control_signal->up;
        #endif

        msg.control_state = last_control_signal->control_state;
        msg.roll = last_control_signal->roll;
        msg.pitch = last_control_signal->pitch;
        msg.yaw = last_control_signal->yaw;
        msg.control_by = last_control_signal->control_by;
        msg.wings_mode = last_control_signal->wings_mode;
    } else {
        msg.control_state = alpha_msgs::msg::ControlInterface::ARM;
        msg.control_by = alpha_msgs::msg::ControlInterface::REACTIVE_OA;
    }

    final_control_PUB->publish(msg);

    #if VISUALIZE
    publishVectorArrow(control_vec_PUB, control_vec, 0.0f, 1.0f, 0.0f); 
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0f, 0.0f, 1.0f); 
    publishVectorArrow(repulsive_vec_PUB, total_repulsive_vec, 1.0f, 0.0f, 0.0f); 
    publishVectorArrow(correction_vec_PUB, correction_vec, 1.0f, 1.0f, 0.0f); 
    #endif

    resetVectors();

#if TIME_ANALYSE
    analyzer->stop_segment("Main Loop");
#endif
}

void spatial_container::ReactiveOANode::inputControlCallback(const alpha_msgs::msg::ControlInterface::SharedPtr msg) {
    lost_control_signal = false;
    lost_control_signal_counter = 0;
    last_control_signal = msg;
    computeControlVector();
}

void spatial_container::ReactiveOANode::perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg) {
    lost_perception = false;
    lost_perception_counter = 0;
    last_perception = msg;
    computeMovementVector();
}

#if VISUALIZE
void spatial_container::ReactiveOANode::publishVectorArrow(
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
    arrow.scale.x = 0.2;  
    arrow.scale.y = 0.2;   
    arrow.scale.z = 0.2;   
    arrow.color.r = r; arrow.color.g = g; arrow.color.b = b; arrow.color.a = 1.0;
    arrow.points.resize(2);
    arrow.points[0].x = 0.0; arrow.points[0].y = 0.0; arrow.points[0].z = 0.0;
    arrow.points[1].x = vec.x(); arrow.points[1].y = vec.y(); arrow.points[1].z = vec.z();
    pub->publish(arrow);
}
#endif