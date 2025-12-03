/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "global_utils/utils.hpp"
// ################################# SYSTEM CONFIG
#define VISUALIZE true

// Time
constexpr float SYSTEM_LOOP_RATE  = 30.0f;   // Hz
constexpr float SYSTEM_LOOP_CYCLE = 1 / SYSTEM_LOOP_RATE;
constexpr int64_t SYSTEM_LOOP_CYCLE_NANOSEC = SYSTEM_LOOP_CYCLE * 1e9;

// ################################# SYSTEM PARAMETER
// Topic path
constexpr const char* CONTROL_INPUT_TOPIC = "/on_drone/drone_control/input/control";
constexpr const char* CONTROL_CORRECTION_TOPIC = "/on_drone/drone_control/correction/control"; // inactive
constexpr const char* CONTROL_FINAL_TOPIC = "/on_drone/drone_control/final/control";
constexpr const char* FUSE_PERCEPTION_TOPIC = "/on_drone/sensor/fuse_perception";
constexpr const char* CONTACT_PARSER_TOPIC = "/on_drone/sensor/body_contact";
constexpr const char* LOGGER_RECORD_TOPIC = "/on_drone/logger/record_control";
constexpr const char* LIDAR_2D_CONTOUR_CLOSE_TOPIC = "/on_drone/sensor/lidar2d/close/contour";
constexpr const char* LIDAR_2D_CONTOUR_FAR_TOPIC = "/on_drone/sensor/lidar2d/far/contour";

// Service path
constexpr const char* CONTROL_WOLRD_GRASSLAND = "/world/grasslands/control";
constexpr const char* CONTROL_WORLD_NAME = CONTROL_WOLRD_GRASSLAND;

// Drone
constexpr float SPEED_MAX_FORWARD = 10.0f;
constexpr float SPEED_MAX_BACKWARD = 10.0f;
constexpr float SPEED_MAX_STRAFE = 10.0f;
constexpr float SPEED_MAX_UP = 8.0f;
constexpr float SPEED_MAX_DOWN = 4.0f;
constexpr float THRUST_SAFE_LIMIT = 0.9f;
constexpr float HOVER_THRUST = -0.5f; // Thrust opposite with moving direction, so in FLU negative = push downward

constexpr float DEGREE = 0.017453292f;

// Sensor
constexpr uint8_t LIDAR_2D_SECTOR_NUM = 12;
constexpr float LIDAR_2D_RANGE_MAX = 30.0f;
constexpr float LIDAR_2D_RANGE_MIN = 0.1f;


// Safety
constexpr float SELF_RADIUS = 1.2f; // radius in meter
constexpr float UNCERTAINTY = 0.05f;
constexpr float SAFE_BUFFER = 0.05f;
constexpr float HAZARD_DISTANCE = SELF_RADIUS + SAFE_BUFFER + UNCERTAINTY;
constexpr float REACT_TIME = 0.033333f; // ms
constexpr float DECELERATE_MAX = 5.0f; // m/s^2
/**
 * Safe bubble calculate by: the 2 time is to make sure UHM I may change that to just normal 1 time
 * SAFE_BUBBLE =  2 * (HAZARD_DISTANCE + speed * REACT_TIME + ((speed * speed) / (2 * DECELERATE_MAX))); 
 */

// Other
constexpr const char* WINDOW_OVERVIEW_FPV = "Alpha FPV";

// ################################# FUNCTION

void setup_for_simulation(rclcpp::Node *node);

