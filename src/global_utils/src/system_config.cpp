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

#pragma region DynamicInfo

// void DynamicInfo::updateENV(const char* env_name) {
//     this->info = "UNSET";
//     const char* env_info = std::getenv(env_name);
//     if(env_info) this->info = std::string(env_info);
// }

// void Name::RECORDED_MANUEVER::update() {
//     this->updateENV("NAME_RECORDED_MANUEVER");
// }

// void Path::RECORD_STORAGE::update() {
//     this->updateENV("RECORD_STORAGE");
// }

#pragma endregion

#pragma region Global functions

void Global::setup_for_simulation(rclcpp::Node *node) {
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

#pragma endregion

Global::Info::Info() {
    // Setup fallback
    drone_name = "error_drone_name";
    world_name = "error_world_name";

    // Get info
    const char* env_drone = std::getenv("DRONE_NAME");
    const char* env_world = std::getenv("WORLD_NAME");

    // Check null
    if(env_drone) drone_name = std::string(env_drone);
    if(env_world) world_name = std::string(env_world);

    // printf(GREEN "Loaded %s in %s" RESET, drone_name.c_str(), world_name.c_str());
}

Global::Info::~Info() {
    // Destructor (can stay empty for this)
}
