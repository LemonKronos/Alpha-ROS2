#include <rclcpp/rclcpp.hpp>
#include "gz_ground_truth/gz_ground_truth.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GzGroundTruthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}