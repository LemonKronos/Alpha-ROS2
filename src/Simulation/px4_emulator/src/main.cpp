#include <rclcpp/rclcpp.hpp>
#include "px4_emulator/px4_emulator.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto px4_emulator_node = std::make_shared<Px4EmulatorNode>();
    rclcpp::spin(px4_emulator_node);
    rclcpp::shutdown();
    return 0;
}