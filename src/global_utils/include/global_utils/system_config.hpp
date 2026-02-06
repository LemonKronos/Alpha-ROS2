/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "global_utils/utils.hpp"

// ################################# SYSTEM CONFIG

#define VISUALIZE true

// Time related settings
namespace Clock {
    constexpr float LOOP_RATE  = 30.0f;   // Hz
    constexpr float LOOP_CYCLE = 1.0f / LOOP_RATE;
    constexpr int64_t LOOP_CYCLE_NANOSEC = LOOP_CYCLE * 1e9;

    constexpr float LOOP_RATE_FAST = 80.0f;
    constexpr float LOOP_CYCLE_FAST = 1.0f / LOOP_RATE_FAST;
    constexpr int64_t LOOP_CYCLE_FAST_NANOSEC = LOOP_CYCLE_FAST * 1e9;

    constexpr float LOOP_RATE_SLOW = 0.5f;
    constexpr float LOOP_CYCLE_SLOW = 1.0f / LOOP_RATE_SLOW;
    constexpr int64_t LOOP_CYCLE_SLOW_NANOSEC = LOOP_CYCLE_SLOW * 1e9;

}

// ################################# SYSTEM PARAMETER

// System thesholds
namespace Threshold {
    constexpr uint8_t MISSED_TOPIC = Clock::LOOP_RATE / 10.0f;
    constexpr uint8_t MISSED_FAST_TOPIC = Clock::LOOP_RATE_FAST / 10.0f;
    constexpr uint8_t MISMATCH_RATE_TOPIC = std::ceil(Clock::LOOP_RATE_FAST / Clock::LOOP_RATE);

}

// Machine file directories
namespace Path {
    constexpr const char* RECORD_ACROBATIC = "/home/mr_lemon/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/obstacle_tunnel";
    constexpr const char* RECORD_ACROBATIC_MANUEVER_NAME = "obstacle_tunnel_demo";

}

// Topic path
namespace Topic {
    constexpr const char* DEPTH_CAM_POINTS = "/alpha_depth_cam/camera/image/points";

    constexpr const char* CONTROL_INPUT = "/on_drone/drone_control/input/control"; // By control signal input Node
    constexpr const char* CONTROL_ACROBATIC = "/on_drone/drone_control/acrobatic/control"; // By Acrobatic OA Node
    constexpr const char* CONTROL_REACTIVE = "/on_drone/drone_control/reactive/control"; // By Reactive OA Node
    constexpr const char* FUSE_PERCEPTION = "/on_drone/sensor/fuse_perception"; // Combine perception info from px4 and sensors
    constexpr const char* CONTACT_PARSER = "/on_drone/sensor/contacts";
    constexpr const char* LOGGER_RECORD = "/on_drone/logger/record_control"; // Contain flag to record data
    constexpr const char* LIDAR_2D_CONTOUR_CLOSE = "/on_drone/sensor/lidar2d/close/contour";
    constexpr const char* LIDAR_2D_CONTOUR_FAR = "/on_drone/sensor/lidar2d/far/contour";
    constexpr const char* LIDAR_3D_URGENT_VOXEL = "/on_drone/sensor/lidar3d/points";
    constexpr const char* OCTO_MAP_RAW = "on_drone/mapping/raw/octomap";
    constexpr const char* SPACIAL_MAP = "on_drone/mapping/spacial/octomap";

}

// Service path
namespace Service {
    constexpr const char* CONTROL_WOLRD_GRASSLAND = "/world/grasslands/control";
    constexpr const char* CONTROL_WOLRD_OBSTACLE_TUNNEL = "/world/obstacle_tunnel/control";
    constexpr const char* CONTROL_WORLD_NAME = CONTROL_WOLRD_OBSTACLE_TUNNEL;

}

namespace Drone {
    constexpr const char* NAME = "alpha_minus_2_0";
    constexpr float WIDTH = 2.144f;
    constexpr float LENGTH = 0.55f;
    constexpr float HEIGHT = 0.05f;
    
    constexpr float SPEED_MAX_FORWARD = 10.0f;
    constexpr float SPEED_MAX_BACKWARD = 10.0f;
    constexpr float SPEED_MAX_STRAFE = 10.0f;
    constexpr float SPEED_MAX_ANGLE = M_PI_2f;
    constexpr float SPEED_MAX_UP = 8.0f;
    constexpr float SPEED_MAX_DOWN = 4.0f;
    constexpr float THRUST_SAFE_LIMIT = 0.9f;
    constexpr float HOVER_THRUST = -0.51f;

    // Safety
    constexpr float RADIUS = 1.2f; // drone radius in meter
    constexpr float UNCERTAINTY = 0.05f; // Tune-able
    constexpr float SAFE_BUFFER = 0.15f; // Tune-able
    constexpr float HAZARD_DISTANCE = RADIUS + SAFE_BUFFER + UNCERTAINTY;
    constexpr float REACT_TIME = Clock::LOOP_CYCLE_FAST; // s
    constexpr float DECELERATE_MAX = 4.0f; // m/s^2

}

namespace Sensor {
    constexpr uint8_t LIDAR_2D_SECTOR_NUM = 12;
    constexpr float LIDAR_2D_RANGE_MAX = 30.0f;
    constexpr float LIDAR_2D_RANGE_MIN = 0.1f;

}

// Other
constexpr const char* WINDOW_OVERVIEW_FPV = "Alpha FPV";

// ################################# FUNCTION
namespace Global {
    void setup_for_simulation(rclcpp::Node *node); // Set up clock sync in simulation

}

