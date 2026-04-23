#include "rclcpp/rclcpp.hpp"
#include "low_spatial/low_spatial.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<alpha_brain::LowSpatial>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}