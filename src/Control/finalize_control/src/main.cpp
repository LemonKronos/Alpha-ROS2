#include "rclcpp/rclcpp.hpp"
#include "finalize_control/finalize_control.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto finalize_ctrl_node = std::make_shared<FinalizeControlNode>();
    rclcpp::spin(finalize_ctrl_node);
    rclcpp::shutdown();
    return 0;
}