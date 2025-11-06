/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#include <cstdint>

constexpr float SYSTEM_LOOP_RATE  = 30.0f;   // Hz
constexpr float SYSTEM_LOOP_CYCLE = 1 / SYSTEM_LOOP_RATE;
constexpr int64_t SYSTEM_LOOP_CYCLE_NANOSEC = SYSTEM_LOOP_CYCLE * 1e9;

constexpr float SPEED_MAX_FORWARD_FW = 10.0f;
constexpr float SPEED_MAX_BACKWARD_FW = 5.0f;
constexpr float SPEED_MAX_STRAFE = 8.0f;
constexpr float SPEED_MAX_UP_FW = 5.0f;
constexpr float SPEED_MAX_DOWN_FW = 2.0f; 
