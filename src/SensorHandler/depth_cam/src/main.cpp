#include "rclcpp/rclcpp.hpp"
#include "depth_cam/depth_cam.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<spatial_container::DepthCamNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}