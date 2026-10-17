#include "foam_visualizer/foam_visualizer.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

FoamVisualizerNode::FoamVisualizerNode(const rclcpp::NodeOptions& options) :
    Node("foam_visualizer", options)
{
    Global::setup_for_simulation(this);

    // Setup TF buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Setup Publisher
    foam_PUB = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualizer/foam",
        10
    );

    loadFoamFromParams();

    // Setup Timer
    loop_TIME = this->create_timer(
        std::chrono::milliseconds(100),
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
    // Verify drone is alive and TF is available
    if (!tf_buffer->canTransform("world", base_link.get(), tf2::TimePointZero, tf2::durationFromSec(0.1))) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    auto stamp = this->get_clock()->now();

    int id = 0;
    for(const auto& sphere : foam) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_link.get();
        marker.header.stamp = stamp;
        marker.ns = "foam";
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

        marker_array.markers.push_back(marker);
    }

    foam_PUB->publish(marker_array);
}