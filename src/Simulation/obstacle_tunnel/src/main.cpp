#include <rclcpp/rclcpp.hpp>
#include "obstacle_tunnel/obstacle_tunnel.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleTunnelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
