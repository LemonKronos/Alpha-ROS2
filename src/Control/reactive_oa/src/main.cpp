#include "rclcpp/rclcpp.hpp"
#include "reactive_oa/reactive_oa.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveOANode>());
    rclcpp::shutdown();
    return 0;
}