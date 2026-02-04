#include "rclcpp/rclcpp.hpp"
#include "octo_map/octo_map.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<OctoMapNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}