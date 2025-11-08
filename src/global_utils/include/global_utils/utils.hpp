#pragma once

#include <limits>
#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

// All q in wxyz
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

    inline Eigen::Quaternionf eulerToQuaternion(const float roll, const float pitch, const float yaw) {
        // Create rotation matrices for each axis
        Eigen::AngleAxisf rollAngle(roll,   Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw,     Eigen::Vector3f::UnitZ());

        // Combine them in ZYX order (yaw * pitch * roll)
        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        q.normalize(); // just in case

        return q;
    }

    inline Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f& rate) {
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(rate.z(), Eigen::Vector3f::UnitZ())  // yaw
        * Eigen::AngleAxisf(rate.y(), Eigen::Vector3f::UnitY())  // pitch
        * Eigen::AngleAxisf(rate.x(), Eigen::Vector3f::UnitX()); // roll
        return Eigen::Quaternionf(R);
    }


    inline Eigen::Quaternionf eulerToQuaternion(const std::array<float, 3>& rate) {
        return eulerToQuaternion(rate[0], rate[1], rate[2]);
    }

    // Returns Vector3f: roll (x), pitch (y), yaw (z) in radians
    inline Eigen::Vector3f quaternionToEuler(const Eigen::Quaternionf& q) {
        Eigen::Vector3f rate;
        float w = q.w(), x = q.x(), y = q.y(), z = q.z();

        // Roll (x-axis rotation)
        rate.x() = std::atan2(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
        // Pitch (y-axis rotation)
        rate.y() = std::asin(std::clamp(2.0f*(w*y - z*x), -1.0f, 1.0f));
        // Yaw (z-axis rotation)
        rate.z() = std::atan2(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

        return rate;
    }

    // Returns Vector3f: roll (x), pitch (y), yaw (z) in radians
    inline Eigen::Vector3f quaternionToEuler(const std::array<float, 4>& q) {
        return quaternionToEuler(Eigen::Quaternionf(q[0], q[1], q[2], q[3]));
    }

    // Returns Vector3f: roll (x), pitch (y), yaw (z) in radians
    inline Eigen::Vector3f quaternionToEuler(const float w, const float x, const float y, const float z) {
        return quaternionToEuler(Eigen::Quaternionf(w, x, y, z));
    }

    // Return std::array<float, 3> north, east, depth
    inline std::array<float, 3> frameFRDtoNED(const float forward, const float right, const float down, const float yaw_W) {
        // 2D rotation by yaw_W (only horizontal plane)
        float c = std::cos(yaw_W);
        float s = std::sin(yaw_W);

        float north = c * forward- s * right;
        float east = s * forward+ c * right;
        float depth = down;

        return {north, east, depth};
    }

    // Return std::array<float, 3> north, east, depth
    inline std::array<float, 3> frameFRDtoNED(const std::array<float, 3> body, const float yaw_W) {
        return frameFRDtoNED(body[0], body[1], body[2], yaw_W);
    }

    inline float quaternionToYaw(const float w, const float x, const float y, const float z) {
        float yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
        return angleInWrapped(yaw);
    }

    inline float quaternionToYaw(const std::array<float, 4> q) {
        return quaternionToYaw(q[0], q[1], q[2], q[3]);
    }

    inline float quaternionToYaw(const Eigen::Quaternionf& q) {
        return quaternionToYaw(q.w(), q.x(), q.y(), q.z());
    }

    inline Eigen::Quaternionf arrayToQuaternion(const std::array<float, 4>& qA) {
        Eigen::Quaternionf qQ(qA[0], qA[1], qA[2], qA[3]);
        return qQ;
    }

    inline std::array<float, 4> quaternionToArray(const Eigen::Quaternionf& qQ) {
        return {qQ.w(), qQ.x(), qQ.y(), qQ.z()};
    }
}
