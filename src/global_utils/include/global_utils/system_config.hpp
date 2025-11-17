/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>

constexpr float SYSTEM_LOOP_RATE  = 30.0f;   // Hz
constexpr float SYSTEM_LOOP_CYCLE = 1 / SYSTEM_LOOP_RATE;
constexpr int64_t SYSTEM_LOOP_CYCLE_NANOSEC = SYSTEM_LOOP_CYCLE * 1e9;

constexpr float SPEED_MAX_FORWARD_FW = 10.0f;
constexpr float SPEED_MAX_BACKWARD_FW = 5.0f;
constexpr float SPEED_MAX_STRAFE = 8.0f;
constexpr float SPEED_MAX_UP_FW = 5.0f;
constexpr float SPEED_MAX_DOWN_FW = 2.0f;
constexpr float THRUST_SAFE_LIMIT = 0.8f;
constexpr float HOVER_THRUST = -0.5f;

constexpr float DEGREE = 0.017453292f;

// Need real data or tinkering
constexpr float SELF_RADIUS = 1.2f; // radius in meter
constexpr float UNCERTAINTY = 0.05f;
constexpr float HAZARD_DISTANCE = SELF_RADIUS + UNCERTAINTY;
constexpr float REACT_TIME = 0.03f; // ms
constexpr float DECELERATE_MAX = 4.0f; // m/s^2
constexpr float SAFE_BUFFER = 0.01f;
