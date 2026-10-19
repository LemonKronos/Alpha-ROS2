#include "reactive_oa/reactive_oa.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    // --- Tuning (stability first) ---
    constexpr float V_ESCAPE = 0.2f;            // constant outward push once inside the push zone
    constexpr float GAMMA = 0.3f;               // weight of the sliding-along-obstacle speed
    constexpr float URGENCY_MAX = 3.0f;
    constexpr float ESDF_RANGE_MARGIN = 1.0f;   // push zone stays this far inside the ESDF max distance
    constexpr float MIN_ZONE_GAP = 0.5f;        // min gap between brake and push radius
    constexpr float TAU_RELEASE = 0.3f;         // s, release time constant of the repulsion filter (attack is instant)
    constexpr int   MAX_CACHE_AGE_TICKS = 15;   // ~150 ms at 100 Hz: how long a cached ESDF read may replace a busy map
}

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
    repulsive_filter.setZero();
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

    sphere_cache.assign(multi_sphere.size(), SphereCache{});
    smoothed_close.assign(multi_sphere.size(), 0.0f);

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

    const float dt = Clock::LOOP_CYCLE_FAST;

    const Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    const Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);
    const Eigen::Quaternionf drone_q_inv = drone_q.inverse();

    // Body-frame speeds in m/s (the geometry below needs real units; vectors elsewhere stay normalized)
    const Eigen::Vector3f v_max(Drone::SPEED_MAX_FORWARD, Drone::SPEED_MAX_STRAFE, Drone::SPEED_MAX_UP);
    const Eigen::Vector3f movement_ms = movement_vec.cwiseProduct(v_max);
    const Eigen::Vector3f control_ms = control_vec.cwiseProduct(v_max);

    // Keep the push zone inside what the ESDF can represent
    const float esdf_max = spatial_container::global_map.esdf_max_distance_m.load(std::memory_order_relaxed);
    const float r_cap = (esdf_max > 0.0f) ? (esdf_max - ESDF_RANGE_MARGIN) : std::numeric_limits<float>::max();

    Eigen::Vector3f repulsive_raw = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < multi_sphere.size(); ++i) {
        const auto& sphere = multi_sphere[i];
        auto& cache = sphere_cache[i];
        float& vc = smoothed_close[i];

        // Slow deflation of the closing speed (also applied when this sphere is skipped)
        auto decay_close_speed = [&]() { vc = std::max(0.0f, vc - Drone::DECELERATE_MAX * dt); };

        const Eigen::Vector3f center_world = drone_pos + (drone_q * sphere.pos);
        float distance = 0.0f;
        Eigen::Vector3f gradient_world = Eigen::Vector3f::Zero();

        // Non-blocking ESDF read with short-lived cache
        switch (spatial_container::global_map.tryGetDistanceAndGradient(center_world, distance, gradient_world)) {
            case spatial_container::QueryResult::Ok:
                cache.valid = true;
                cache.age_ticks = 0;
                cache.distance = distance;
                cache.gradient_world = gradient_world;
                cache.center_at_read = center_world;
                break;

            case spatial_container::QueryResult::Unobserved:
                cache.valid = false;      // no data here: an old value no longer describes this spot
                decay_close_speed();
                continue;

            case spatial_container::QueryResult::Busy:
                if (!cache.valid || ++cache.age_ticks > MAX_CACHE_AGE_TICKS) {
                    decay_close_speed();
                    continue;             // nothing recent enough to trust
                }
                gradient_world = cache.gradient_world;
                // Moving along the gradient since the read changes the distance accordingly
                distance = cache.distance +
                    (center_world - cache.center_at_read).dot(cache.gradient_world.normalized());
                break;
        }

        if (gradient_world.hasNaN() || gradient_world.norm() < 1e-4f || std::isnan(distance)) {
            decay_close_speed();
            continue;
        }

        // Unit vector pointing AWAY from the obstacle, in body frame
        const Eigen::Vector3f gradient_body = drone_q_inv * gradient_world.normalized();

        // Closing speed toward THIS obstacle (m/s): flying parallel to a surface gives ~0
        const float v_close_target = std::max(0.0f, std::max(
            movement_ms.dot(-gradient_body),   // actual motion
            control_ms.dot(-gradient_body)     // pilot intent
        ));

        // Fast inflate, slow deflate
        if (v_close_target > vc) vc = v_close_target;
        else vc = std::max(v_close_target, vc - Drone::DECELERATE_MAX * dt);

        // Zone radii, kept continuous and inside the ESDF range
        const float r_hazard = Drone::hazard_distance(sphere.radius);
        float r_brake = Drone::braking_distance(vc, r_hazard);
        float r_push = Drone::pushing_distance(vc, r_brake);

        r_brake = std::max(r_hazard, std::min(r_brake, r_cap - MIN_ZONE_GAP));
        r_push = std::max(r_brake + MIN_ZONE_GAP, std::min(r_push, r_cap));

        if (distance >= r_push) continue;

        // Urgency: continuous 0 -> 1 -> 2 -> 3
        float urgency = 0.0f;
        if (distance > r_brake) {
            urgency = (r_push - distance) / std::max(0.01f, r_push - r_brake);                 // 0..1
        } else if (distance > r_hazard) {
            urgency = 1.0f + ((r_brake - distance) / std::max(0.01f, r_brake - r_hazard));    // 1..2
        } else {
            urgency = std::min(URGENCY_MAX, 2.0f + ((r_hazard - distance) / std::max(0.01f, r_hazard))); // 2..3
        }

        const float v_in = std::max(0.0f, movement_vec.dot(-gradient_body));
        const float i_in = std::max(0.0f, control_vec.dot(-gradient_body));
        const float v_slip = std::sqrt(std::max(0.0f, movement_vec.squaredNorm() - std::pow(movement_vec.dot(gradient_body), 2.0f)));

        const float m_repel = urgency * (i_in + v_in + V_ESCAPE + (GAMMA * v_slip));

        repulsive_raw += gradient_body * m_repel;

        #if DEBUG
        RCLCPP_INFO(this->get_logger(),
            YELLOW "Sphere %zu | Dist: %.2f | v_close: %.2f | Bounds [Push: %.2f, Brake: %.2f, Haz: %.2f] | Urgency: %.2f | m_repel: %.2f" RESET,
            i, distance, vc, r_push, r_brake, r_hazard, urgency, m_repel);
        #endif
    }

    // Bound the magnitude without changing the direction
    const float raw_norm = repulsive_raw.norm();
    if (raw_norm > 1.0f) repulsive_raw /= raw_norm;

    // Fast attack, slow release: react instantly to growing danger, relax gently
    if (repulsive_raw.norm() >= repulsive_filter.norm()) {
        repulsive_filter = repulsive_raw;
    } else {
        repulsive_filter += (dt / (TAU_RELEASE + dt)) * (repulsive_raw - repulsive_filter);
    }
    repulsive_vec = repulsive_filter;

    #if TIME_ANALYSE
        analyzer->stop_segment("Compute Repulsive Vector");
    #endif
}

void spatial_container::ReactiveOANode::computeReactiveVector() {
    reactive_vec = control_vec + repulsive_vec;

    if (reactive_vec.hasNaN()) {
        reactive_vec.setZero();
        return;
    }

    // Scale uniformly so no axis exceeds 1.0 (clamping per axis would rotate the vector toward the obstacle)
    const float max_axis = reactive_vec.cwiseAbs().maxCoeff();
    if (max_axis > 1.0f) reactive_vec /= max_axis;

    #if DEBUG && 1
    RCLCPP_INFO(
        this->get_logger(),
        BLUE
        "intent= %.2f; movement= %.2f; repulsive= %.2f; reactive= %.2f"
        RESET,
        control_vec.norm(), movement_vec.norm(), repulsive_vec.norm(), reactive_vec.norm()
    );
    #endif
}

void spatial_container::ReactiveOANode::resetVectors() {
    if(lost_control_signal) control_vec.setZero();
    if(lost_perception) {
        movement_vec.setZero();
        repulsive_filter.setZero();
    }
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