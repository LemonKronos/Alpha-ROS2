#include "reactive_oa/reactive_oa.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    // --- Speed governor tuning (stability first) ---
    constexpr float A_BRAKE = Drone::DECELERATE_MAX;   // m/s^2 deceleration the governor may rely on (be conservative)
    constexpr float T_REACT = 0.4f;             // s, loop + latency + control lag
    constexpr float K_BRAKE = 1.0f;             // extra braking per m/s of speed above the allowed closing speed
    constexpr float V_BRAKE_MAX = 3.0f;         // m/s, strongest "reverse" the governor will command while braking
    constexpr float V_ESCAPE_MAX = 1.5f;        // m/s, max outward speed when inside the hazard sphere
    constexpr float K_ESCAPE = 1.5f;            // 1/s, outward speed per meter of penetration
    constexpr float ESDF_RANGE_MARGIN = 1.0f;   // governor range stays this far inside the ESDF max distance
    constexpr float EDGE_BLEND = 2.0f;          // m, fade-in of the governor at the edge of that range
    constexpr float REPULSIVE_MAX = 1.5f;       // normalized magnitude limit (>1 so it can cancel a full command and still brake)
    constexpr float TAU_RELEASE = 0.3f;         // s, release time constant of the output filter (attack is instant)
    constexpr int   MAX_CACHE_AGE_TICKS = 30;   // ~300 ms at 100 Hz: cached read may replace a BUSY map this long
    constexpr int   UNOBSERVED_GRACE_TICKS = 10;// ~100 ms: cached read may replace an UNOBSERVED query this long
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

    RCLCPP_INFO(this->get_logger(), GREEN "Loaded %zu FOAM spheres" RESET, multi_sphere.size());
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

//TODO Make this 30 Hz 
void spatial_container::ReactiveOANode::extractAndPublishSpatialBoxes() {
    #if TIME_ANALYSE
        analyzer->start_segment("Extract Spatial Boxes");
    #endif

    if(spatial_obs_PUB->get_subscription_count() == 0) return;

    auto obs_msg = alpha_msgs::msg::SpatialObservation();
    obs_msg.header.stamp = this->now();
    obs_msg.header.frame_id = this->base_link.get();

    Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);
    Eigen::Matrix3f body_rot = drone_q.toRotationMatrix();

    // Coarse Box Physical Parameters
    float coarse_stride = (10.0f * Drone::UNIT_SIZE) / Sensor::SPATIAL_BOX_DIM;
    float coarse_ref_dist = std::sqrt(3.0f * std::pow(5.0f * Drone::UNIT_SIZE, 2));

    // Detail Box Physical Parameters
    float intent_norm = std::clamp(control_vec.norm() / Drone::SPEED_MAX_FORWARD, 0.0f, 1.0f);
    float l_v = (5.0f + 15.0f * intent_norm) * Drone::UNIT_SIZE;
    float detail_stride_x = l_v / Sensor::SPATIAL_BOX_DIM;
    float detail_stride_yz = (5.0f * Drone::UNIT_SIZE) / Sensor::SPATIAL_BOX_DIM;
    float detail_ref_dist = std::sqrt(std::pow(l_v, 2) + 2.0f * std::pow(2.5f * Drone::UNIT_SIZE, 2));

    // Align X with control_vec, preserving the drone's body roll
    Eigen::Matrix3f intent_rot = body_rot; 
    if (!control_vec.isZero(1e-3f)) {
        Eigen::Vector3f intent_x = body_rot * control_vec.normalized();
        
        // Cross the drone's actual Body Z-axis with Intent X to get Intent Y
        Eigen::Vector3f body_z = body_rot.col(2); 
        Eigen::Vector3f intent_y = body_z.cross(intent_x).normalized();
        
        // Fallback if intent_x is perfectly aligned with body_z (e.g., commanding straight up/down relative to the deck)
        if(intent_y.hasNaN() || !intent_y.isZero(1e-3f)) {
            intent_y = body_rot.col(1); // Use body Y as fallback
        }
        
        Eigen::Vector3f intent_z = intent_x.cross(intent_y).normalized();
        
        intent_rot.col(0) = intent_x;
        intent_rot.col(1) = intent_y;
        intent_rot.col(2) = intent_z;
    }

    int idx = 0;
    
    // Sub-sample offsets (-0.25 and +0.25 of the stride from the center of the cell)
    const float sub_offsets[2] = {-0.25f, 0.25f};

    // Loop Z, Y, X (Channel-First flattening)
    for (int z = 0; z < Sensor::SPATIAL_BOX_DIM; ++z) {
        for (int y = 0; y < Sensor::SPATIAL_BOX_DIM; ++y) {
            for (int x = 0; x < Sensor::SPATIAL_BOX_DIM; ++x) {
                
                // Aggregators for the 2x2x2 volumetric sampling
                float c_sum = 0.0f, d_sum = 0.0f;
                float d_min = std::numeric_limits<float>::max();
                int c_valid = 0, d_valid = 0;

                // 2x2x2 Sub-sampling Loop
                for(float sz : sub_offsets) {
                    for(float sy : sub_offsets) {
                        for(float sx : sub_offsets) {
                            
                            // Coarse sub-point
                            Eigen::Vector3f c_local(
                                (x - Sensor::SPATIAL_BOX_DIM/2.0f + 0.5f + sx) * coarse_stride, 
                                (y - Sensor::SPATIAL_BOX_DIM/2.0f + 0.5f + sy) * coarse_stride, 
                                (z - Sensor::SPATIAL_BOX_DIM/2.0f + 0.5f + sz) * coarse_stride
                            );
                            Eigen::Vector3f c_world = drone_pos + body_rot * c_local;
                            float c_dist = 0.0f;
                            if(global_map.getDistance(c_world, c_dist)) {
                                c_sum += c_dist;
                                c_valid++;
                            }

                            // Detail sub-point
                            Eigen::Vector3f d_local(
                                (x + 0.5f + sx) * detail_stride_x, 
                                (y - Sensor::SPATIAL_BOX_DIM/2.0f + 0.5f + sy) * detail_stride_yz, 
                                (z - Sensor::SPATIAL_BOX_DIM/2.0f + 0.5f + sz) * detail_stride_yz
                            );
                            Eigen::Vector3f d_world = drone_pos + intent_rot * d_local;
                            float d_dist = 0.0f;
                            if(global_map.getDistance(d_world, d_dist)) {
                                d_sum += d_dist;
                                d_min = std::min(d_min, d_dist);
                                d_valid++;
                            }
                        }
                    }
                }

                // Channel 0: Coarse Mean ESDF | Channel 1: Coarse Mask
                obs_msg.coarse_box[idx]        = (c_valid > 0) ? ((c_sum / c_valid) / coarse_ref_dist) : 1.0f;
                obs_msg.coarse_box[1728 + idx] = (c_valid > 0) ? 1.0f : 0.0f;
                
                // Channel 0: Detail Min ESDF | Channel 1: Detail Mean ESDF | Channel 2: Detail Mask
                obs_msg.detail_box[idx]        = (d_valid > 0) ? (d_min / detail_ref_dist) : 1.0f;
                obs_msg.detail_box[1728 + idx] = (d_valid > 0) ? ((d_sum / d_valid) / detail_ref_dist) : 1.0f;
                obs_msg.detail_box[3456 + idx] = (d_valid > 0) ? 1.0f : 0.0f;

                idx++;
            }
        }
    }

    spatial_obs_PUB->publish(obs_msg);

    #if TIME_ANALYSE
        analyzer->stop_segment("Extract Spatial Boxes");
    #endif
}

//TODO Make this 30 Hz
void spatial_container::ReactiveOANode::computeMultiSphereGradient() {
    #if TIME_ANALYSE
        analyzer->start_segment("Compute Repulsive Vector");
    #endif

    const float dt = Clock::LOOP_CYCLE_FAST;

    const Eigen::Vector3f drone_pos(last_perception->position[0], last_perception->position[1], last_perception->position[2]);
    const Eigen::Quaternionf drone_q = frame_utils::arrayToQuaternion(last_perception->q);
    const Eigen::Quaternionf drone_q_inv = drone_q.inverse();

    // Body-frame speeds in m/s (the governor needs real units; vectors elsewhere stay normalized)
    const Eigen::Vector3f v_max(Drone::SPEED_MAX_FORWARD, Drone::SPEED_MAX_STRAFE, Drone::SPEED_MAX_UP);
    const Eigen::Vector3f movement_ms = movement_vec.cwiseProduct(v_max);
    const Eigen::Vector3f control_ms = control_vec.cwiseProduct(v_max);

    // Range in which the ESDF is trustworthy
    const float esdf_max = spatial_container::global_map.esdf_max_distance_m.load(std::memory_order_relaxed);
    const float r_cap = (esdf_max > 0.0f) ? (esdf_max - ESDF_RANGE_MARGIN) : std::numeric_limits<float>::max();

    #if DEBUG
    static unsigned n_ok = 0, n_busy = 0, n_unobs = 0, n_skipped = 0;
    #endif

    Eigen::Vector3f repulsive_raw = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < multi_sphere.size(); ++i) {
        const auto& sphere = multi_sphere[i];
        auto& cache = sphere_cache[i];

        const Eigen::Vector3f center_world = drone_pos + (drone_q * sphere.pos);
        float distance = 0.0f;
        Eigen::Vector3f gradient_world = Eigen::Vector3f::Zero();
        bool have_data = false;

        // Replace a missing read with the last good one, moved along its gradient by the drone's motion
        auto use_cache = [&]() {
            gradient_world = cache.gradient_world;
            distance = cache.distance +
                (center_world - cache.center_at_read).dot(cache.gradient_world.normalized());
            have_data = true;
        };

        //_ Non-blocking ESDF read with short-lived cache
        switch (spatial_container::global_map.tryGetDistanceAndGradient(center_world, distance, gradient_world)) {
            case spatial_container::QueryResult::Ok:
                cache.valid = true;
                cache.age_ticks = 0;
                cache.distance = distance;
                cache.gradient_world = gradient_world;
                cache.center_at_read = center_world;
                have_data = true;
                #if DEBUG
                ++n_ok;
                #endif
                break;

            case spatial_container::QueryResult::Unobserved:
                #if DEBUG
                ++n_unobs;
                #endif
                if (cache.valid && ++cache.age_ticks <= UNOBSERVED_GRACE_TICKS) use_cache();
                else cache.valid = false;
                break;

            case spatial_container::QueryResult::Busy:
                #if DEBUG
                ++n_busy;
                #endif
                if (cache.valid && ++cache.age_ticks <= MAX_CACHE_AGE_TICKS) use_cache();
                break;
        }

        if (!have_data || gradient_world.hasNaN() || gradient_world.norm() < 1e-4f || std::isnan(distance)) {
            #if DEBUG
            ++n_skipped;
            #endif
            continue;
        }
        if (distance >= r_cap) continue;   // beyond the range the ESDF represents

        // Unit vector pointing AWAY from the obstacle, in body frame
        const Eigen::Vector3f g_body = drone_q_inv * gradient_world.normalized();

        const float r_hazard = Drone::hazard_distance(sphere.radius);
        const float clearance = distance - r_hazard;   // m, negative = inside the hazard sphere

        //_ Maximum closing speed from which we can still stop before the hazard sphere:
        //   v_allow = -a*T + sqrt((a*T)^2 + 2*a*clearance). Depends only on distance, not on our own speed.
        float v_allow = 0.0f;
        if (clearance > 0.0f) {
            const float at = A_BRAKE * T_REACT;
            v_allow = std::sqrt(at * at + 2.0f * A_BRAKE * clearance) - at;
        }
        v_allow = std::min(v_allow, Drone::SPEED_MAX_FORWARD);

        // Closing speeds toward this obstacle (m/s): commanded and actual. Parallel motion gives ~0.
        const float c = control_ms.dot(-g_body);
        const float v = movement_ms.dot(-g_body);

        //_ Closing speed we allow the command to have
        float u = std::min(c, v_allow);
        if (v > v_allow) u = std::min(u, v_allow - K_BRAKE * (v - v_allow));            // already too fast: brake
        if (clearance < 0.0f) u = std::min(u, -std::min(V_ESCAPE_MAX, K_ESCAPE * -clearance)); // inside: ease out
        u = std::max(u, -V_BRAKE_MAX);

        // Fade the governor in over the last EDGE_BLEND meters of the ESDF range (no step at the edge)
        const float edge = std::clamp((r_cap - distance) / EDGE_BLEND, 0.0f, 1.0f);

        // Reduction of the closing component, applied along the outward gradient (never fights a retreating pilot)
        const float delta = std::max(0.0f, c - u) * edge;
        repulsive_raw += (g_body * delta).cwiseQuotient(v_max);    // back to normalized command units

        #if DEBUG
        RCLCPP_INFO(this->get_logger(),
            YELLOW "Sphere %zu | Dist: %.2f | clear: %.2f | cmd_close: %.2f | act_close: %.2f | v_allow: %.2f | delta: %.2f" RESET,
            i, distance, clearance, c, v, v_allow, delta);
        #endif
    }

    #if DEBUG
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        GRAY "ESDF reads ok=%u busy=%u unobserved=%u skipped=%u" RESET, n_ok, n_busy, n_unobs, n_skipped);
    #endif

    //_ Bound the magnitude without changing the direction
    const float raw_norm = repulsive_raw.norm();
    if (raw_norm > REPULSIVE_MAX) repulsive_raw *= (REPULSIVE_MAX / raw_norm);

    //_ Fast attack, slow release
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