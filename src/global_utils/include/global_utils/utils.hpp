#pragma once

#include <limits>
#include <array>
#include <cmath>

/* ######################################## Color */
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PINK    "\033[35m"
#define TEAL    "\033[36m"
#define RESET   "\033[0m"

/* ######################################## Constances */
constexpr float NO_DATA_f = std::numeric_limits<float>::quiet_NaN();
constexpr float NO_DATA_d = std::numeric_limits<float>::quiet_NaN();

/* ########################################## Function*/
namespace frame_utils {
    inline float angleInWrapped(float angle) {
        while(angle < -M_PI) angle += 2*M_PI;
        while(angle > M_PI) angle -= 2*M_PI;
        return angle;
    }

    inline float angleInPolar(float angle) {
        while(angle < 0) angle += 2*M_PI;
        while(angle > 2*M_PI) angle -=2*M_PI;
        return angle;
    }

    // q in x, y, z, w
    inline std::array<float, 4> euler_to_quaternion(const float roll, const float pitch, const float yaw) {
        float c_roll = cosf(roll/2);
        float s_roll = sinf(roll/2);
        float c_pitch = cosf(pitch/2);
        float s_pitch = sinf(pitch/2);
        float c_yaw = cosf(yaw/2);
        float s_yaw = sinf(yaw/2);

        std::array<float, 4> Aq;
            
        Aq[0] = s_roll*c_pitch*c_yaw - c_roll*s_pitch*s_yaw;
        Aq[1] = c_roll*s_pitch*c_yaw + s_roll*c_pitch*s_yaw;
        Aq[2] = c_roll*c_pitch*s_yaw - s_roll*s_pitch*c_yaw;
        Aq[3] = c_roll*c_pitch*c_yaw + s_roll*s_pitch*s_yaw;

        return Aq;
    }

    // q in x, y, z, w
    inline std::array<float, 4> euler_to_quaternion(const std::array<float, 3>& rate) {
        return euler_to_quaternion(rate[0], rate[1], rate[2]);
    }

    // q in x, y, z, w
    inline std::array<float, 3> quaternion_to_euler(const float x, const float y, const float z, const float w) {
        std::array<float, 3> euler;

        // roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f)
            euler[1] = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
        else
            euler[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    // q in x, y, z, w
    inline std::array<float, 3> quaternion_to_euler(const std::array<float, 4> q) {
        return quaternion_to_euler(q[0], q[1],q[2], q[3]);
    }

    inline std::array<float, 3> frame_FRD_to_NED(const float forward, const float right, const float down, const float yaw_W) {
        // 2D rotation by yaw_W (only horizontal plane)
        float c = std::cos(yaw_W);
        float s = std::sin(yaw_W);

        float north = c * forward- s * right;
        float east = s * forward+ c * right;
        float depth = down;

        return {north, east, depth};
    }

    inline std::array<float, 3> frame_FRD_to_NED(const std::array<float, 3> body, const float yaw_W) {
        return frame_FRD_to_NED(body[0], body[1], body[2], yaw_W);
    }

    // q in x, y, z, w
    inline float quaternion_to_yaw(const float x, const float y, const float z, const float w) {
        float yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
        return angleInWrapped(yaw);
    }

    // q in x, y, z, w
    inline float quaternion_to_yaw(const std::array<float, 4> q) {
        return quaternion_to_yaw(q[0], q[1], q[2], q[3]);
    }
}
