#include "rclcpp/rclcpp.hpp"
#include "map_visualizer/map_visualizer.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<alpha_brain::MapVisualizerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}