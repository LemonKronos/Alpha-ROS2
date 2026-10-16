#include "rclcpp/rclcpp.hpp"
#include "reactive_oa/reactive_oa.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<spatial_container::ReactiveOANode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}