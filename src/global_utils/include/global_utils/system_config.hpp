/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "global_utils/utils.hpp"

// ################################# SYSTEM CONFIG

#define VISUALIZE true
#define DO_REACTIVE_OA true

// Time related settings
namespace Clock {
    constexpr float LOOP_CYCLE = 0.031f;
    constexpr float LOOP_RATE  = 1.0f / LOOP_CYCLE;
    constexpr int64_t LOOP_CYCLE_NANOSEC = LOOP_CYCLE * 1e9;

    constexpr float LOOP_CYCLE_FAST = 0.013f;
    constexpr float LOOP_RATE_FAST = 1.0f / LOOP_CYCLE_FAST;
    constexpr int64_t LOOP_CYCLE_FAST_NANOSEC = LOOP_CYCLE_FAST * 1e9;

    constexpr float LOOP_CYCLE_SLOW = 1.997f;
    constexpr float LOOP_RATE_SLOW = 1.0f / LOOP_CYCLE_SLOW;
    constexpr int64_t LOOP_CYCLE_SLOW_NANOSEC = LOOP_CYCLE_SLOW * 1e9;

    constexpr float LOOP_CYCLE_HEAVY = 4.001f;
    constexpr float LOOP_RATE_HEAVY = 1.0f / LOOP_CYCLE_HEAVY;
    constexpr int64_t LOOP_CYCLE_HEAVY_NANOSEC = LOOP_CYCLE_HEAVY * 1e9;

}

// ################################# SYSTEM PARAMETER

// Dynamic info: data taken from env, json, yaml, etc ...
class DynamicInfo {
public:
    DynamicInfo(std::function<std::string()> logic);
    inline const char* get() { return info.c_str(); }
    void update();
protected:
    std::function<std::string()> update_logic;
    std::string info;
};
namespace DynamicInfoHelper {
    std::string updateENV(const char* env_name);
}

namespace Name {
    namespace Dynamic {
        class DRONE : public DynamicInfo { public: DRONE(); };
        class BASE_LINK : public DynamicInfo { public: BASE_LINK(); };
        class WORLD : public DynamicInfo { public: WORLD(); };
        class RECORDED_MANUEVER : public DynamicInfo { public: RECORDED_MANUEVER(); };

    }
}

// System thesholds
namespace Threshold {
    constexpr uint8_t MISSED_TOPIC = Clock::LOOP_RATE / 10.0f;
    constexpr uint8_t MISSED_FAST_TOPIC = Clock::LOOP_RATE_FAST / 10.0f;
    constexpr uint8_t MISMATCH_RATE_TOPIC = std::ceil(Clock::LOOP_RATE_FAST / Clock::LOOP_RATE);

}

// Machine file directories
namespace Path {
    namespace Dynamic {
        class RECORD_STORAGE : public DynamicInfo { public: RECORD_STORAGE(); };

    }

}

// Topic path
namespace Topic {
    constexpr const char* DEPTH_CAM_FRONT_PL = "/sensor/depth_cam/front/points";
    constexpr const char* DEPTH_CAM_LEFT_PL = "/sensor/depth_cam/left/points";
    constexpr const char* DEPTH_CAM_RIGHT_PL = "/sensor/depth_cam/right/points";
    constexpr const char* RGB_CAM_FRONT = "/sensor/rgb_cam/camera/image";
    constexpr const char* OVERVIEW_CAM = "/sensor/overview_cam/camera/image";
    constexpr const char* LIDAR_2D_AROUND_SCAN = "/sensor/lidar_2d/scan";
    constexpr const char* LIDAR_1D_DOWN_SCAN = "/sensor/lidar_1d_down/scan";
    constexpr const char* BODY_CONTACT = "/sensor/contact_body/contact";
    constexpr const char* ROTOR_0_CONTACT = "/sensor/contact_rotor0/contact";
    constexpr const char* ROTOR_1_CONTACT = "/sensor/contact_rotor1/contact";
    constexpr const char* ROTOR_2_CONTACT = "/sensor/contact_rotor2/contact";
    constexpr const char* ROTOR_3_CONTACT = "/sensor/contact_rotor3/contact";

    constexpr const char* CONTROL_INPUT = "/internal/drone_control/input/control"; // By control signal input Node
    constexpr const char* CONTROL_ACROBATIC = "/internal/drone_control/acrobatic/control"; // By Acrobatic OA Node
    constexpr const char* CONTROL_REACTIVE = "/internal/drone_control/reactive/control"; // By Reactive OA Node
    constexpr const char* FUSE_PERCEPTION = "/internal/sensor/fuse_perception"; // Combine perception info from px4 and sensors
    constexpr const char* CONTACT_PARSER = "/internal/sensor/contacts";
    constexpr const char* LOGGER_RECORD = "/internal/logger/record_control"; // Contain flag to record data
    constexpr const char* LIDAR_2D_CONTOUR_CLOSE = "/internal/sensor/lidar2d/close/contour";
    constexpr const char* LIDAR_2D_CONTOUR_FAR = "/internal/sensor/lidar2d/far/contour";
    constexpr const char* VFH_HAZARD_SEEING = "/internal/mapping/hazard/seeing/vfh";
    constexpr const char* VFH_HAZARD_MEMORY = "/internal/mapping/hazard/memory/vfh";
    
}

// Service path
namespace Service {
    constexpr const char* WORLD_CONTROL = "/world/control";
    constexpr const char* WORLD_SPAWN = "/world/create";
    constexpr const char* WORLD_KILL = "/world/remove";
    constexpr const char* WORLD_SET_POSE = "/world/set_pose";

}

namespace Drone {
    constexpr float WIDTH = 2.15f;
    constexpr float LENGTH = 0.93f;
    constexpr float HEIGHT = 0.31f;
    
    constexpr float SPEED_MAX_FORWARD = 10.0f;
    constexpr float SPEED_MAX_BACKWARD = 10.0f;
    constexpr float SPEED_MAX_STRAFE = 10.0f;
    constexpr float SPEED_MAX_ANGLE = M_PI_2f;
    constexpr float SPEED_MAX_UP = 8.0f;
    constexpr float SPEED_MAX_DOWN = 4.0f;
    constexpr float THRUST_SAFE_LIMIT = 0.9f;
    constexpr float HOVER_THRUST = 0.51f;

    // Safety
    constexpr float RADIUS = 1.2f; // drone radius in meter
    constexpr float UNCERTAINTY = 0.05f; // Tune-able
    constexpr float SAFE_BUFFER = 0.15f; // Tune-able
    constexpr float HAZARD_DISTANCE = RADIUS + SAFE_BUFFER + UNCERTAINTY;
    constexpr float REACT_TIME = Clock::LOOP_CYCLE; // s
    constexpr float DECELERATE_MAX = 1.0f; // m/s^2
    // hazard_distance = Drone::HAZARD_DISTANCE + speed * Drone::REACT_TIME + (speed_sq / (2 * Drone::DECELERATE_MAX));
    // About 14m at the speed of 5m/s

    // Body box anti clipping, independent with sensor mount point, sensor data have to be body frame transformed
    constexpr float MAX_X = 0.53f + UNCERTAINTY;
    constexpr float MIN_X = -0.4f - UNCERTAINTY;
    constexpr float MAX_Y = 1.075f + UNCERTAINTY;
    constexpr float MIN_Y = -1.075f - UNCERTAINTY;
    constexpr float MAX_Z = 0.21f + UNCERTAINTY;
    constexpr float MIN_Z = -0.1f -UNCERTAINTY; 
}

namespace Sensor {
    constexpr uint8_t LIDAR_2D_SECTOR_NUM = 12;
    constexpr float LIDAR_2D_RANGE_MAX = 30.0f;
    constexpr float LIDAR_2D_RANGE_MIN = 0.1f;
    constexpr float DEPTH_CAM_RANGE = 30.0f;
    constexpr float OCTREE_VOXEL_RESOLUTION = 0.5f;
    constexpr float VFH_RESOLUTION = 5.0f * DEGREE;
    constexpr int VFH_AZIMUTH_BINS = 2.0f * M_PI / VFH_RESOLUTION;
    constexpr int VFH_LATITUDE_BINS = M_PI / VFH_RESOLUTION;
    constexpr int VFH_TOTAL_BINS = VFH_AZIMUTH_BINS * VFH_LATITUDE_BINS;
    constexpr int VFH_MSG_BIT_SIZE = 32;
    constexpr int VFH_MSG_CHUNK_SIZE = std::ceil(VFH_TOTAL_BINS / VFH_MSG_BIT_SIZE);

}

namespace Window {
    constexpr const char* OVERVIEW_FPV = "Alpha FPV";
}

// ################################# FUNCTION
namespace Global {
    void setup_for_simulation(rclcpp::Node *node); // Set up clock sync in simulation
    
    // class Info {
    // private:
    //     std::string drone_name;
    //     std::string world_name;
    // public:
    //     Info();
    //     ~Info();
    //     std::string getDroneName() { return drone_name;};
    //     std::string getWorldName() { return world_name;};
    // };
}

