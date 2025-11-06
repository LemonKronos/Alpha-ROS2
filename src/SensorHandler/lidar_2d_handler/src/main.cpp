#include "rclcpp/rclcpp.hpp"
#include "lidar_2d_handler/lidar2d.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto lidar_2d_handler_node = std::make_shared<Lidar2dHandlerNode>();
    rclcpp::spin(lidar_2d_handler_node);
    rclcpp::shutdown();
    return 0;
}