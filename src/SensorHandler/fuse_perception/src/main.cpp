#include "rclcpp/rclcpp.hpp"
#include "fuse_perception/fuse_perception.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto fuse_perception_node = std::make_shared<FusePerceptionNode>();
    rclcpp::spin(fuse_perception_node);
    rclcpp::shutdown();
    return 0;
}