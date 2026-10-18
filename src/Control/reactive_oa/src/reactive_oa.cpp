#include "reactive_oa/reactive_oa.hpp"

spatial_container::ReactiveOANode::ReactiveOANode(const rclcpp::NodeOptions &options) :
Node("reactive_oa_node", options)
{
    Global::setup_for_simulation(this);

    //: Load Params
    loadFoamFromParams();
    
    //: Publishers
    final_control_PUB = this->create_publisher<alpha_msgs::msg::ControlInterface>(Topic::CONTROL_REACTIVE, rclcpp::SensorDataQoS());
    spatial_obs_PUB = this->create_publisher<alpha_msgs::msg::SpatialObservation>(Topic::OBSERVATION_BOXES, rclcpp::SensorDataQoS());

    #if VISUALIZE
        control_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/control_vector", 10);
        movement_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/movement_vector", 10);
        repulsive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/repulsive_vector", 10);
        reactive_vec_PUB = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/correction_vector", 10);
    #endif

    //: Subscribers
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

    //: Timer
    node_loop_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&ReactiveOANode::nodeLoopCallback, this)
    );

    //: Init variables
    control_vec.setZero();
    movement_vec.setZero();
    repulsive_vec.setZero();
    reactive_vec.setZero();

    lost_control_signal = true;
    lost_control_signal_counter = Threshold::ALLOW_MISSED_FAST_TOPIC;
    lost_perception = true;
    lost_perception_counter = Threshold::ALLOW_MISSED_FAST_TOPIC;

    #if TIME_ANALYSE
    analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger(), 800);
    #endif
}

spatial_container::ReactiveOANode::~ReactiveOANode(){
#if TIME_ANALYSE
    analyzer->printSummary();
#endif
}

void spatial_container::ReactiveOANode::loadFoamFromParams() {
    const auto values = this->declare_parameter<std::vector<double>>("foam_spheres", std::vector<double>{});

    if(values.empty() || values.size() % 4 != 0) {
        multi_sphere.push_back({
            Eigen::Vector3f::Zero(),
            Drone::RADIUS
        });

        RCLCPP_ERROR(this->get_logger(), RED "Unable to load FOAM, replacing with single bubble" RESET);
    }

    multi_sphere.reserve(values.size() / 4);

    for(std::size_t i = 0; i < values.size(); i += 4) {
        const float radius = static_cast<float>(values[i + 3]);

        if(!std::isfinite(radius) || radius <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), YELLOW "Contain FOAM with incorrect radius, skipped" RESET);
            continue;
        }

        multi_sphere.push_back({
            Eigen::Vector3f(
                static_cast<float>(values[i]),
                static_cast<float>(values[i + 1]),
                static_cast<float>(values[i + 2])
            ),
            radius
        });
    }

    RCLCPP_INFO(this->get_logger(), GREEN "Loaded %d FOAM spheres" RESET, multi_sphere.size());
}

void spatial_container::ReactiveOANode::computeControlVector() {
    control_vec = Eigen::Vector3f(
        last_control_signal->forward,
        last_control_signal->left,
        last_control_signal->up
    ); 
}

void spatial_container::ReactiveOANode::computeMovementVector() {
    auto norm = [] (float speed, float max_speed) -> float {
        return std::clamp(speed / max_speed, -1.0f, 1.0f);
    };

    Eigen::Vector3f world_vel(
        norm(last_perception->velocity[0], Drone::SPEED_MAX_FORWARD),
        norm(last_perception->velocity[1], Drone::SPEED_MAX_STRAFE),
        norm(last_perception->velocity[2], Drone::SPEED_MAX_UP)
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
        analyzer->start_segment("Compute Repulsive Vector");
    #endif
    
    repulsive_vec.setZero();
    Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);

    // 1. Anticipated Speed Blending
    float physical_speed = movement_vec.norm();
    float intent_speed = control_vec.norm(); // control_vec is already scaled to m/s
    float target_speed = std::max(physical_speed, intent_speed);

    // 2. Asymmetric Rate Limiting (Fast Inflate, Slow Deflate)
    float dt = Clock::LOOP_CYCLE_FAST; // Assuming this is defined as seconds (e.g., 0.01f)
    if (target_speed > smoothed_speed_) {
        // Inflate instantly to guarantee safety against sudden intent spikes
        smoothed_speed_ = target_speed;
    } else {
        // Deflate slowly constrained by the drone's maximum physical deceleration
        smoothed_speed_ -= Drone::DECELERATE_MAX * dt;
        if (smoothed_speed_ < target_speed) smoothed_speed_ = target_speed;
    }

    for (size_t i = 0; i < multi_sphere.size(); ++i) {
        const auto& sphere = multi_sphere[i];
        
        // Use the smoothed_speed_ instead of actual_speed for geometry scaling
        float r_hazard = Drone::hazard_distance(sphere.radius); 
        float r_brake = Drone::braking_distance(smoothed_speed_, r_hazard);
        float r_push = Drone::pushing_distance(smoothed_speed_, r_brake);

        Eigen::Vector3f center_world = drone_pos + (drone_q * sphere.pos);
        float distance = 0.0f;
        Eigen::Vector3f gradient_world = Eigen::Vector3f::Zero();

        bool observed = spatial_container::global_map.getDistanceAndGradient(center_world, distance, gradient_world); 

        if(!observed) continue;

        #if DEBUG
        RCLCPP_INFO(
            this->get_logger(),
            BLUE "ESDF Distance = %.2f" RESET,
            distance
        );
        #endif

        if (distance < r_push) {
            float urgency = 0.0f;
            if (distance > r_brake) {
                urgency = (r_push - distance) / std::max(0.01f, r_push - r_brake);
            } else if (distance > r_hazard) {
                urgency = 1.0f + ((r_brake - distance) / std::max(0.01f, r_brake - r_hazard));
            } else {
                urgency = 3.0f; 
            }

            Eigen::Vector3f gradient_body = drone_q.inverse() * gradient_world.normalized();
            
            float v_in = std::max(0.0f, movement_vec.dot(-gradient_body));
            float i_in = std::max(0.0f, control_vec.dot(-gradient_body));
            float v_slip = std::sqrt(std::max(0.0f, movement_vec.squaredNorm() - std::pow(movement_vec.dot(gradient_body), 2.0f)));

            constexpr float V_ESCAPE = 0.2f; 
            constexpr float GAMMA = 0.3f;    
            
            float m_repel = urgency * (i_in + v_in + V_ESCAPE + (GAMMA * v_slip));
            
            repulsive_vec += gradient_body * m_repel;

            #if DEBUG
            // Log exactly which sphere is triggering, its spatial boundaries, and the resulting urgency
            RCLCPP_INFO(this->get_logger(),
                YELLOW "Sphere %zu | Dist: %.2f | Bounds [Push: %.2f, Brake: %.2f, Haz: %.2f] | Urgency: %.2f | m_repel: %.2f" RESET,
                i, distance, r_push, r_brake, r_hazard, urgency, m_repel);
            #endif
        }
    }

    #if TIME_ANALYSE
        analyzer->stop_segment("Compute Repulsive Vector");
    #endif
}

void spatial_container::ReactiveOANode::computeReactiveVector() {
    reactive_vec = control_vec + repulsive_vec;
    
    reactive_vec.x() = std::clamp(reactive_vec.x(), -1.0f, 1.0f);
    reactive_vec.y() = std::clamp(reactive_vec.y(), -1.0f, 1.0f);
    reactive_vec.z() = std::clamp(reactive_vec.z(), -1.0f, 1.0f);
}

void spatial_container::ReactiveOANode::resetVectors() {
    if(lost_control_signal) control_vec.setZero();
    if(lost_perception) movement_vec.setZero();
    reactive_vec.setZero();
    repulsive_vec.setZero();
}

void spatial_container::ReactiveOANode::nodeLoopCallback() {
    #if TIME_ANALYSE
        analyzer->start_segment("Node Loop");
    #endif

    // Update statuses
    if(lost_perception_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) lost_perception_counter++;
    lost_perception = (lost_perception_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC);

    if(lost_control_signal_counter < Threshold::ALLOW_MISSED_FAST_TOPIC) lost_control_signal_counter++;
    lost_control_signal = (lost_control_signal_counter >= Threshold::ALLOW_MISSED_FAST_TOPIC);

    if (!lost_perception) {
        // extractAndPublishSpatialBoxes();
        computeMultiSphereGradient();
    }

    computeReactiveVector();

    // Publish Final Control
    auto msg = alpha_msgs::msg::ControlInterface();
    msg.header.stamp = this->now();
    
    if(!lost_control_signal) {
        #if DO_REACTIVE_OA
        msg.forward = reactive_vec.x();
        msg.left = reactive_vec.y();
        msg.up = reactive_vec.z();
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
    publishVectorArrow(control_vec_PUB, control_vec, 0.0f, 1.0f, 0.0f); // Green
    publishVectorArrow(movement_vec_PUB, movement_vec, 0.0f, 0.0f, 1.0f); // Blue
    publishVectorArrow(repulsive_vec_PUB, repulsive_vec, 1.0f, 0.0f, 0.0f); // Red
    publishVectorArrow(reactive_vec_PUB, reactive_vec, 1.0f, 1.0f, 0.0f); // Yellow
    #endif

    resetVectors();

#if TIME_ANALYSE
    analyzer->stop_segment("Node Loop");
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
    float r, float g, float b
) {
    // De-norm the vector
    Eigen::Vector3f real_value;
    real_value.x() = vec.x() * Drone::SPEED_MAX_FORWARD;
    real_value.y() = vec.y() * Drone::SPEED_MAX_STRAFE;
    real_value.z() = vec.z() * Drone::SPEED_MAX_UP;

    // Make marker
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
    arrow.points[1].x = real_value.x(); arrow.points[1].y = real_value.y(); arrow.points[1].z = real_value.z();
    pub->publish(arrow);
}
#endif