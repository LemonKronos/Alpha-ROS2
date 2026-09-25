#include "rclcpp/rclcpp.hpp"
#include "finalize_control/finalize_control.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto finalize_ctrl_node = std::make_shared<FinalizeControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(finalize_ctrl_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}