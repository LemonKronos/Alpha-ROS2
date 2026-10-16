#include "rclcpp/rclcpp.hpp"
#include "omni_lidar/omni_lidar.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<spatial_container::OmniLidarNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}