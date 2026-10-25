#include "foam_visualizer/foam_visualizer.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

FoamVisualizerNode::FoamVisualizerNode(const rclcpp::NodeOptions& options) :
    Node("foam_visualizer", options)
{
    Global::setup_for_simulation(this);

    // Subcribers
    perception_SUB = this->create_subscription<alpha_msgs::msg::FusePerception>(
        Topic::FUSE_PERCEPTION,
        rclcpp::SensorDataQoS(),
        std::bind(&FoamVisualizerNode::perceptionCallback, this, _1)
    );

    // Setup Publisher
    static_foam_PUB = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualizer/foam/static",
        10
    );
    inflated_foam_PUB = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualizer/foam/inflated",
        10
    );

    loadFoamFromParams();

    // Setup Timer
    loop_TIME = this->create_timer(
        std::chrono::nanoseconds(Clock::LOOP_CYCLE_FAST_NANOSEC),
        std::bind(&FoamVisualizerNode::TimerCallback, this)
    );
}

FoamVisualizerNode::~FoamVisualizerNode() {}

void FoamVisualizerNode::loadFoamFromParams() {
    const auto values = this->declare_parameter<std::vector<double>>("foam_spheres", std::vector<double>{});
    
    if(values.empty() || values.size() % 4 != 0) {
        foam.push_back({
            Eigen::Vector3f::Zero(),
            Drone::RADIUS
        });
        RCLCPP_ERROR(this->get_logger(), "Unable to load FOAM, replacing with single bubble");
    }

    foam.reserve(values.size() / 4);

    for(std::size_t i = 0; i < values.size(); i += 4) {
        const float radius = static_cast<float>(values[i + 3]);

        if(!std::isfinite(radius) || radius <= 0.0f) {
            RCLCPP_WARN(this->get_logger(), "Contain FOAM with incorrect radius, skipped");
            continue;
        }

        foam.push_back({
            Eigen::Vector3f(
                static_cast<float>(values[i]),
                static_cast<float>(values[i + 1]),
                static_cast<float>(values[i + 2])
            ),
            radius
        });
    }

    RCLCPP_INFO(this->get_logger(), GREEN "Loaded %zu FOAM spheres" RESET, foam.size());
}

void FoamVisualizerNode::TimerCallback() {
    auto stamp = this->get_clock()->now();

    // Static foam
    if(static_foam_PUB->get_subscription_count() != 0) {
        visualization_msgs::msg::MarkerArray static_marker_array;

        int id = 0;
        for(const auto& sphere : foam) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = base_link.get();
            marker.header.stamp = stamp;
            marker.ns = "foam_static";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = sphere.pos.x();
            marker.pose.position.y = sphere.pos.y();
            marker.pose.position.z = sphere.pos.z();
            
            marker.pose.orientation.w = 1.0;

            // Individual sphere radius
            marker.scale.x = sphere.radius * 2.0; 
            marker.scale.y = sphere.radius * 2.0;
            marker.scale.z = sphere.radius * 2.0;

            // Solid semi-blue white
            marker.color.r = 0.8f;
            marker.color.g = 0.8f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;

            static_marker_array.markers.push_back(marker);
        }

        static_foam_PUB->publish(static_marker_array);
    }

    // Inflated foam
    if(inflated_foam_PUB->get_subscription_count() != 0) {
        visualization_msgs::msg::MarkerArray inflated_marker_array;
        int id = 0;
        for(const auto& sphere : foam) {

            float r_hazard = Drone::hazard_distance(sphere.radius);
            float r_brake = Drone::braking_distance(smoothed_speed, r_hazard);
            float r_push = Drone::pushing_distance(smoothed_speed, r_brake);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = base_link.get();
            marker.header.stamp = stamp;
            marker.ns = "foam_inflated";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = sphere.pos.x();
            marker.pose.position.y = sphere.pos.y();
            marker.pose.position.z = sphere.pos.z();
            
            marker.pose.orientation.w = 1.0;

            // Individual sphere radius
            marker.scale.x = r_push * 2.0; 
            marker.scale.y = r_push * 2.0;
            marker.scale.z = r_push * 2.0;

            // Transparent yellow
            marker.color.r = 0.8f;
            marker.color.g = 0.8f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;

            inflated_marker_array.markers.push_back(marker);
        }

        inflated_foam_PUB->publish(inflated_marker_array);
    }
}

void FoamVisualizerNode::perceptionCallback(const alpha_msgs::msg::FusePerception::SharedPtr msg) {
    auto norm = [] (float speed, float max_speed) -> float {
        return std::clamp(speed / max_speed, -1.0f, 1.0f);
    };

    Eigen::Vector3f world_vel(
        norm(msg->velocity[0], Drone::SPEED_MAX_FORWARD),
        norm(msg->velocity[1], Drone::SPEED_MAX_STRAFE),
        norm(msg->velocity[2], Drone::SPEED_MAX_UP)
    );
    if(world_vel.hasNaN()) world_vel.setZero();

    // This was copied from Reactive OA

    // 1. Anticipated Speed Blending
    float physical_speed = world_vel.norm();

    // 2. Asymmetric Rate Limiting (Fast Inflate, Slow Deflate)
    float dt = Clock::LOOP_CYCLE_FAST; // Assuming this is defined as seconds (e.g., 0.01f)
    if (physical_speed > smoothed_speed) {
        // Inflate instantly to guarantee safety against sudden intent spikes
        smoothed_speed = physical_speed;
    } else {
        // Deflate slowly constrained by the drone's maximum physical deceleration
        smoothed_speed -= Drone::DECELERATE_MAX * dt;
        if (smoothed_speed < physical_speed) smoothed_speed = physical_speed;
    }
}