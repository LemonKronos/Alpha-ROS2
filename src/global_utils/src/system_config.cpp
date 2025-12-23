#include "global_utils/system_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include <future>

/**
 * @brief This use for automatically choosing clock for node that need a node loop clock, return either simulation clock or realtime hardware clock
 */
#include <thread>
#include <chrono>
#include <map>
#include <string>
#include <vector>

void setup_for_simulation(rclcpp::Node *node) {
    // 1. Check override parameter first
    bool current_val = false;
    if (node->get_parameter("use_sim_time", current_val)) {
        if (current_val) return; 
    }

    // RCLCPP_INFO(node->get_logger(), PINK "Auto check if node run in simulation..." RESET);

    // 2. Loop Wait (Polling the Graph)
    // We try for 3 seconds. We do NOT spin, we just peek at the network graph.
    // DDS background threads usually populate this even without spinning.
    bool clock_found = false;
    int retries = 30; // 3 seconds

    while (retries > 0) {
        // Get list of all topics currently known
        std::map<std::string, std::vector<std::string>> topics = node->get_topic_names_and_types();

        if (topics.find("/clock") != topics.end()) {
            clock_found = true;
            break;
        }

        // Wait a bit for DDS discovery to update
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        retries--;
    }

    // 3. Apply Setting
    if (clock_found) {
        if (node->has_parameter("use_sim_time")) {
             node->set_parameter(rclcpp::Parameter("use_sim_time", true));
        } else {
             node->declare_parameter("use_sim_time", true);
        }
        RCLCPP_WARN(node->get_logger(), YELLOW "Node run using simulation clock!" RESET);
    } else {
        // Default to Realtime if not found
        RCLCPP_INFO(node->get_logger(), PINK "No simulation clock found, run in realtime" RESET);
        if (!node->has_parameter("use_sim_time")) {
             node->declare_parameter("use_sim_time", false);
        }
    }
}
