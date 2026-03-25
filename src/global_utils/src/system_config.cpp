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

DynamicInfo::DynamicInfo(std::function<std::string()> logic) 
    : update_logic(logic), info("UNSET") {
    
    if (update_logic) {
        info = update_logic(); // Auto-runs on construction!
    }
}

void DynamicInfo::update() {
    if (update_logic) {
        info = update_logic();
    }
}

std::string DynamicInfoHelper::updateENV(const char* env_name) {
    const char* env_info = std::getenv(env_name);
    if(env_info) {
        // printf(GREEN "Get ENV success %s = %s\n" RESET, env_name, env_info);
        return std::string(env_info);
    }
    else {
        printf(RED "Get ENV error %s\n" RESET, env_name);
        return "UNSET";
    }
}

Name::Dynamic::DRONE::DRONE() : DynamicInfo([]() -> std::string {
    return DynamicInfoHelper::updateENV("DRONE_NAME") + "_0";
}) {}

Name::Dynamic::BASE_LINK::BASE_LINK() : DynamicInfo([]() -> std::string {
    Name::Dynamic::DRONE drone_name;
    return std::string(drone_name.get()) + "/base_link";
}) {}

Name::Dynamic::WORLD::WORLD() : DynamicInfo([]() -> std::string {
    return DynamicInfoHelper::updateENV("WOLRD_NAME");
}) {}

Name::Dynamic::RECORDED_MANUEVER::RECORDED_MANUEVER() : DynamicInfo([]() -> std::string {
    return DynamicInfoHelper::updateENV("RECORDED_MANUEVER_NAME");
}) {}

Path::Dynamic::RECORD_STORAGE::RECORD_STORAGE() : DynamicInfo([]() -> std::string {
    return DynamicInfoHelper::updateENV("RECORD_STORAGE_PATH");
}) {}


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

// Global::Info::Info() {
//     // Setup fallback
//     drone_name = "error_drone_name";
//     world_name = "error_world_name";

//     // Get info
//     const char* env_drone = std::getenv("DRONE_NAME");
//     const char* env_world = std::getenv("WORLD_NAME");

//     // Check null
//     if(env_drone) drone_name = std::string(env_drone);
//     if(env_world) world_name = std::string(env_world);

//     // printf(GREEN "Loaded %s in %s" RESET, drone_name.c_str(), world_name.c_str());
// }

// Global::Info::~Info() {
//     // Destructor (can stay empty for this)
// }
