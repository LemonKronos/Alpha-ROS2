/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "global_utils/utils.hpp"

// ################################# CONFIG PARAMETER

constexpr float SYSTEM_LOOP_RATE  = 30.0f;   // Hz
constexpr float SYSTEM_LOOP_CYCLE = 1 / SYSTEM_LOOP_RATE;
constexpr int64_t SYSTEM_LOOP_CYCLE_NANOSEC = SYSTEM_LOOP_CYCLE * 1e9;

constexpr float SPEED_MAX_FORWARD = 10.0f;
constexpr float SPEED_MAX_BACKWARD = 5.0f;
constexpr float SPEED_MAX_STRAFE = 8.0f;
constexpr float SPEED_MAX_UP = 5.0f;
constexpr float SPEED_MAX_DOWN = 2.0f;
constexpr float THRUST_SAFE_LIMIT = 0.8f;
constexpr float HOVER_THRUST = -0.525f; // Thrust opposite with moving direction, so in FLU negative = push downward

constexpr float DEGREE = 0.017453292f;

// Need real data or tinkering
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

// ################################# FUNCTION

void setup_for_simulation(rclcpp::Node *node);

