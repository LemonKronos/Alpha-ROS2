#include "global_utils/system_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include <future>

/**
 * @brief This use for automatically choosing clock for node that need a node loop clock, return either simulation clock or realtime hardware clock
 */
void setup_for_simulation(rclcpp::Node *node) {
    // 1. Check if the user manually set the param already. If so, respect it.
    bool current_val = false;
    if (node->get_parameter("use_sim_time", current_val)) {
        if (current_val) return; 
    }

    // 2. Create a temporary detector node
    auto detector_node = rclcpp::Node::make_shared("clock_detector_temp");
    
    // 3. Create a Promise to signal when we get a message
    auto prom = std::make_shared<std::promise<void>>();
    auto future = prom->get_future();

    // 4. Subscribe to /clock
    auto sub = detector_node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 
        rclcpp::SensorDataQoS(),
        [&prom](const rosgraph_msgs::msg::Clock::SharedPtr) {
            prom->set_value(); 
        });

    // 5. Spin the detector for up to 2.0 seconds (Blocking)
    auto status = rclcpp::spin_until_future_complete( // spin_until_future_complete will wait until 'prom->set_value()' is called OR timeout
        detector_node, 
        future, 
        std::chrono::seconds(2)
    );

    // 6. Logic
    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        if (node->has_parameter("use_sim_time")) { // Set node to use simulation clock
             node->set_parameter(rclcpp::Parameter("use_sim_time", true));
        } else {
             node->declare_parameter("use_sim_time", true);
        }
        RCLCPP_WARN(node->get_logger(), PINK "Node run using simulation clock!" RESET);
    } else { // Fallback to realtime clock just in case
        if (!node->has_parameter("use_sim_time")) {
             node->declare_parameter("use_sim_time", false);
        }
    }
}
