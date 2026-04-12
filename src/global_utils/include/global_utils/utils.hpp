#pragma once

#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <array>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <numeric>
#include <iomanip>
#include <mutex>
#include <sstream>

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
constexpr float DEGREE = 0.017453292f;

/* ########################################## Function*/

// All q in Eigen::Quaternionf wxyz
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
    inline Eigen::Vector3f quaternionToEuler(const std::array<float, 4>& q) {
        return quaternionToEuler(Eigen::Quaternionf(q[0], q[1], q[2], q[3]));
    }
    inline Eigen::Vector3f quaternionToEuler(const float w, const float x, const float y, const float z) {
        return quaternionToEuler(Eigen::Quaternionf(w, x, y, z));
    }

    // Return std::array<float, 3> East, North, Up
    inline std::array<float, 3> frameFLUtoENU(float forward, float left, float up, float yaw_rad) {
        float c = cosf(yaw_rad);
        float s = sinf(yaw_rad);
        float E = forward * c - left * s;
        float N = forward * s + left * c;
        return {E, N, up};
    }
    inline std::array<float, 3> frameFLUtoENU(const std::array<float, 3> body, const float yaw_W) {
        return frameFLUtoENU(body[0], body[1], body[2], yaw_W);
    }

    // From PX4 NED frame to ROS2 ENU frame
    inline std::array<float, 3> frameNEDtoENU(const std::array<float, 3>& pos) {
        return {pos[1], pos[0] , -pos[2]};
    }

    // From ROS2 ENU frame to PX4 NED frame
    inline std::array<float, 3> frameENUtoNED(const std::array<float, 3>& pos) {
        return {pos[1], pos[0], -pos[2]};
    }

    // From PX4 FRD frame to ROS2 FLU frame
    inline std::array<float, 3> frameFRDtoFLU(const float forward, const float right, const float down) {
        return {forward, -right, -down};
    }
    inline std::array<float, 3> frameFRDtoFLU(const std::array<float, 3>& vec) {
        return frameFRDtoFLU(vec[0], vec[1], vec[2]);
    }
    
    // From ROS2 FLU frame to PX4 FRD frame
    inline std::array<float, 3> frameFLUtoFRD(const float forward, const float left, const float up) {
        return {forward, -left, -up};
    }
    inline std::array<float, 3> frameFLUtoFRD(const std::array<float, 3>& vec) {
        return frameFLUtoFRD(vec[0], vec[1], vec[2]);
    }
    inline std::array<float, 3> frameFLUtoFRD(const Eigen::Vector3f& vec) {
        return frameFLUtoFRD(vec.x(), vec.y(), vec.z());
    }

    // Quaternion wxyz from PX4 NED frame to ROS2 ENU frame
    inline Eigen::Quaternionf quaternionNEDtoENU(const Eigen::Quaternionf& q) {
        Eigen::Quaternionf q_tf_world(0, M_SQRT1_2f, M_SQRT1_2f, 0);
        Eigen::Quaternionf q_tf_body(0.0f, 1.0f, 0.0f, 0.0f);
        return q_tf_world * q * q_tf_body;
    }
    inline Eigen::Quaternionf quaternionNEDtoENU(const std::array<float, 4>& qA) {
        Eigen::Quaternionf qQ(qA[0], qA[1], qA[2], qA[3]);
        Eigen::Quaternionf q_transform(0, M_SQRT1_2f, M_SQRT1_2f, 0);
        return q_transform * qQ;
    }
    
    // Quaternion wxyz from ROS2 ENU frame to PX4 NED frame
    inline Eigen::Quaternionf quaternionENUtoNED(const Eigen::Quaternionf& q) {
        Eigen::Quaternionf q_tf_world(0, M_SQRT1_2f, M_SQRT1_2f, 0);
        Eigen::Quaternionf q_tf_body(0.0f, 1.0f, 0.0f, 0.0f);
        return q_tf_world * q * q_tf_body;
    }

    inline float quaternionToYaw(const float w, const float x, const float y, const float z) {
        float yaw = static_cast<float>(std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)));
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

} // namespace frame_utils

namespace math_utils {
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

    inline Eigen::Vector3f toSpherical(const Eigen::Vector3f C_vector) {
        return Eigen::Vector3f(
            std::atan2(C_vector.y(), C_vector.x()),
            std::atan2(-C_vector.z(), C_vector.head<2>().norm()),
            C_vector.norm()
        );
    }

    inline Eigen::Vector3f toCartesian(const Eigen::Vector3f S_vector) {
        float distance = S_vector.z();
        float distance_cos_pitch = distance * std::cos(S_vector.y());

        return Eigen::Vector3f(
            distance_cos_pitch * std::cos(S_vector.x()),
            distance_cos_pitch * std::sin(S_vector.x()),
            -distance * std::sin(S_vector.y())
        );
    }

    template<typename T>
    inline T linearMap(const T input, const T in_min, const T in_max, const T out_min, const T out_max) {
        if(in_min == in_max) return (out_max - out_min)/2;
        else if(in_min < in_max) {
            T ratio = (input - in_min) / (in_max - in_min);
            ratio = std::clamp(ratio, 0.0f, 1.0f);
            return out_min + ratio*(out_max - out_min);
        }
        else {
            T ratio = (input - in_max) / (in_min - in_max);
            ratio = std::clamp(ratio, 0.0f, 1.0f);
            return out_min + (1 - ratio)*(out_max - out_min);
        }
    }

    template <typename T>
    inline T expoMap(const T input, const T in_min, const T in_max, const T out_min, const T out_max, const T sensitivity) {
        if (in_max <= in_min)
            return out_min;

        T ratio = (input - in_min) / (in_max - in_min);
        ratio = std::clamp(ratio, 0.0f, 1.0f);

        T expo_ratio = std::pow(ratio, sensitivity);
        return out_min + expo_ratio * (out_max - out_min);
    }
} // namespace math_utils

namespace time_utils {
    class TimeAnalyzer {
    private:
        struct TagData {
            std::vector<double> history;
            size_t head = 0;           // Circular buffer index
            size_t count = 0;          // How many samples we currently have
            double min_time = 1e9;     // Init ridiculously high
            double max_time = 0.0;
            double last_time = 0.0;    // Store the most recent compute time
            std::chrono::high_resolution_clock::time_point start_time;
            
            // Constructor pre-allocates the vector so we never dynamically allocate during flight
            TagData(size_t window_size) : history(window_size, 0.0) {}
        };

        rclcpp::Logger logger;
        std::unordered_map<std::string, TagData> data_;
        size_t window_size_;
        std::mutex mtx_; // Thread safety for ROS2 callbacks

    public:
        // Default window size of 500 loops. Adjust as needed.
        TimeAnalyzer(rclcpp::Logger logger, size_t window_size = 500) : logger(logger), window_size_(window_size) {}

        // 1. Call this right before the code you want to measure
        void start_segment(const std::string& tag) {
            std::lock_guard<std::mutex> lock(mtx_);
            auto it = data_.find(tag);

            // If this tag doesn't exist yet, emplace it (only allocates memory on the very first call)
            if (it == data_.end()) it = data_.emplace(tag, TagData(window_size_)).first;
            it->second.start_time = std::chrono::high_resolution_clock::now();
        }

        // 2. Call this right after. Returns the microseconds it took just in case you need it logically.
        double stop_segment(const std::string& tag) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::lock_guard<std::mutex> lock(mtx_);
            
            auto it = data_.find(tag);
            if (it == data_.end()) return 0.0; // Failsafe if stop() is called before start()

            auto& d = it->second;
            double duration_us = std::chrono::duration<double, std::micro>(end_time - d.start_time).count();
            
            // Update circular buffer
            d.history[d.head] = duration_us;
            d.head = (d.head + 1) % window_size_;
            if (d.count < window_size_) d.count++;
            
            // Update global bounds
            d.min_time = std::min(d.min_time, duration_us);
            d.max_time = std::max(d.max_time, duration_us);
            d.last_time = duration_us;

            return duration_us;
        }

        // 3. Print the absolute latest time for a specific loop
        void printCurrent(const std::string& tag) {
            std::lock_guard<std::mutex> lock(mtx_);
            auto it = data_.find(tag);
            if (it != data_.end()) {
                RCLCPP_INFO_STREAM(
                    logger, 
                    "[Profiler] " << tag << " took: " << std::fixed << std::setprecision(2) << it->second.last_time << " us\n"
                );
            }
        }

        // Convenience function to stop and print instantly
        void stopAndPrint(const std::string& tag) {
            stop_segment(tag);
            printCurrent(tag);
        }

        // 4. The grand finale. Call this in your Node Destructor.
        void printSummary() {
            std::lock_guard<std::mutex> lock(mtx_);
            std::stringstream ss;
            ss  << "\n====================== COMPUTE PROFILER SUMMARY (us) ======================\n"
                << std::left << std::setw(20) << "Tag" 
                << std::right << std::setw(12) << "Avg" 
                << std::setw(12) << "Med" 
                << std::setw(12) << "Min" 
                << std::setw(12) << "Max" 
                << std::setw(10) << "Samples" << "\n"
                << "----------------------------------------------------------------------\n";

            for (auto& [tag, d] : data_) {
                if (d.count == 0) continue;

                // Calculate Average
                double sum = std::accumulate(d.history.begin(), d.history.begin() + d.count, 0.0);
                double avg = sum / d.count;

                // Calculate Median (O(N) time, incredibly fast)
                std::vector<double> valid_history(d.history.begin(), d.history.begin() + d.count);
                size_t n = valid_history.size();
                std::nth_element(valid_history.begin(), valid_history.begin() + n / 2, valid_history.end());
                
                double median = valid_history[n / 2];
                // If even number of elements, technically median is the average of the two middle ones
                if (n % 2 == 0) {
                    auto max_it = std::max_element(valid_history.begin(), valid_history.begin() + n / 2);
                    median = (median + *max_it) / 2.0;
                }
                ss  << std::left << std::setw(20) << tag 
                    << std::right << std::fixed << std::setprecision(2)
                    << std::setw(12) << avg 
                    << std::setw(12) << median 
                    << std::setw(12) << d.min_time 
                    << std::setw(12) << d.max_time 
                    << std::setw(10) << d.count << "\n";
            }
            ss << "======================================================================\n";
            RCLCPP_INFO_STREAM(logger, ss.str());
        }
    };
} // namespace time_utils