#include <rclcpp/rclcpp.hpp>
#include "foam_visualizer/foam_visualizer.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FoamVisualizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}