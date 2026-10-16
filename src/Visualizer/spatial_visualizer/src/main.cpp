#include "rclcpp/rclcpp.hpp"
#include "spatial_visualizer/spatial_visualizer.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<spatial_container::SpatialVisualizerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}