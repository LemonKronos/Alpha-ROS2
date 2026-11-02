/**
 * @brief This file is used for system dynamic configuration, like loop rate, sensor specification, etc.
 */

#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "global_utils/utils.hpp"

//_                                   SYSTEM CONFIG

#define ALLOW_DEBUG true

// Time related settings
namespace Clock {
    constexpr float LOOP_RATE  = 30.0f;
    constexpr float LOOP_CYCLE = 1.0f / LOOP_RATE;
    constexpr uint64_t LOOP_CYCLE_NANOSEC = static_cast<uint64_t>(LOOP_CYCLE * 1e9);

    constexpr float LOOP_RATE_FAST = 100.0f;
    constexpr float LOOP_CYCLE_FAST = 1.0f / LOOP_RATE_FAST;
    constexpr uint64_t LOOP_CYCLE_FAST_NANOSEC = static_cast<uint64_t>(LOOP_CYCLE_FAST * 1e9);

    constexpr float LOOP_RATE_SLOW = 10.0f;
    constexpr float LOOP_CYCLE_SLOW = 1.0f / LOOP_RATE_SLOW;
    constexpr uint64_t LOOP_CYCLE_SLOW_NANOSEC = static_cast<uint64_t>(LOOP_CYCLE_SLOW * 1e9);

    constexpr float LOOP_CYCLE_HEAVY = 4.001f;
    constexpr float LOOP_RATE_HEAVY = 1.0f / LOOP_CYCLE_HEAVY;
    constexpr uint64_t LOOP_CYCLE_HEAVY_NANOSEC = static_cast<uint64_t>(LOOP_CYCLE_HEAVY * 1e9);

}

//_                                   SYSTEM PARAMETER

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
        class DRONE_ID : public DynamicInfo { public : DRONE_ID(); };
        class BASE_LINK : public DynamicInfo { public: BASE_LINK(); };
        class WORLD : public DynamicInfo { public: WORLD(); };
        class RECORDED_MANUEVER : public DynamicInfo { public: RECORDED_MANUEVER(); };

    }
}

// System thesholds
namespace Threshold {
    constexpr uint8_t ALLOW_MISSED_TOPIC = 3ul;
    constexpr int64_t ALLOW_MISSED_TOPIC_ns = ALLOW_MISSED_TOPIC * Clock::LOOP_CYCLE_NANOSEC;

    constexpr uint8_t ALLOW_MISSED_FAST_TOPIC = std::ceil(Clock::LOOP_RATE_FAST / 10.0f);
    constexpr int64_t ALLOW_MISSED_FAST_TOPIC_ns = ALLOW_MISSED_FAST_TOPIC * Clock::LOOP_CYCLE_FAST_NANOSEC;

    constexpr uint8_t ALLOW_MISSED_NORMAL_TO_FAST_TOPIC = ALLOW_MISSED_TOPIC * ALLOW_MISSED_FAST_TOPIC; //* Soon to be deprecated


    constexpr int64_t TF_TIMEOUT_ns = Clock::LOOP_CYCLE_NANOSEC * 6;
    constexpr int64_t TF_TIMEOUT_FAST_ns = Clock::LOOP_CYCLE_FAST_NANOSEC * 8;

    constexpr float ENTER_ATTIUDE_MODE = 0.01f;
    
    constexpr uint8_t OFFBOARD_STREAM = 20;

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
    constexpr const char* LIDAR_2D_AROUND_PL = "/sensor/lidar_2d/scan/points";
    constexpr const char* LIDAR_1D_DOWN_PL = "/sensor/lidar_1d_down/scan/points";
    constexpr const char* BODY_CONTACT = "/sensor/contact_body/contact";
    constexpr const char* ROTOR_0_CONTACT = "/sensor/contact_rotor0/contact";
    constexpr const char* ROTOR_1_CONTACT = "/sensor/contact_rotor1/contact";
    constexpr const char* ROTOR_2_CONTACT = "/sensor/contact_rotor2/contact";
    constexpr const char* ROTOR_3_CONTACT = "/sensor/contact_rotor3/contact";

    constexpr const char* CONTROL_INPUT = "/internal/drone_control/input/control"; // By control signal input Node
    constexpr const char* CONTROL_ACROBATIC = "/internal/drone_control/acrobatic/floats"; // By Acrobatic OA Node
    constexpr const char* CONTROL_REACTIVE = "/internal/drone_control/reactive/control"; // By Reactive OA Node

    constexpr const char* FUSE_PERCEPTION = "/internal/sensor/fuse_perception"; // Combine perception info from px4 and sensors
    constexpr const char* CONTACT_PARSER = "/internal/sensor/contacts";

    constexpr const char* LOGGER_RECORD = "/internal/logger/record_control"; // Contain flag to record data
    
    constexpr const char* SPATIAL_OBSERVATION = "/internal/sensor/observation_boxes"; // For Acrobatic OA 

    constexpr const char* DRONE_MOTORS = "/drone/motors";
    constexpr const char* DRONE_ELEVON_LEFT = "/drone/elevon_left";
    constexpr const char* DRONE_ELEVON_RIGHT = "/drone/elevon_right";
    constexpr const char* DRONE_TILT_RIGHT = "/drone/tilt_right";
    constexpr const char* DRONE_TILT_LEFT = "/drone/tilt_left";
    constexpr const char* DRONE_ODOMETRY = "/drone/odometry";

    namespace Px4 {
        constexpr const char* VEHICLE_ODOMETRY = "/fmu/out/vehicle_odometry";
        constexpr const char* VEHICLE_STATUS = "/fmu/out/vehicle_status_v1";
        constexpr const char* VEHICLE_LAND_DETECTED = "/fmu/out/vehicle_land_detected";
        constexpr const char* VEHICLE_COMMAND = "/fmu/in/vehicle_command";
        constexpr const char* VEHICLE_ATTITUDE_SETPOINT = "/fmu/in/vehicle_attitude_setpoint";
        constexpr const char* OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode";
        constexpr const char* TRAJECTORY_SETPOINT = "/fmu/in/trajectory_setpoint";
        constexpr const char* ACTUATOR_MOTORS = "/fmu/in/actuator_motors";
        constexpr const char* ACTUATOR_SERVOS = "/fmu/in/actuator_servos";

    }
    
}

// Service path
namespace Service {
    constexpr const char* WORLD_CONTROL = "/world/control";
    constexpr const char* WORLD_SPAWN = "/world/create";
    constexpr const char* WORLD_KILL = "/world/remove";
    constexpr const char* WORLD_SET_POSE = "/world/set_pose";

}

namespace Drone {
    constexpr float UNIT_SIZE = 1.0f;
    constexpr float WIDTH = 2.15f;
    constexpr float LENGTH = 0.93f;
    constexpr float HEIGHT = 0.31f;
    
    constexpr float SPEED_MAX_FORWARD = 10.0f;
    constexpr float SPEED_MAX_BACKWARD = 6.0f;
    constexpr float SPEED_MAX_STRAFE = 10.0f;
    constexpr float SPEED_MAX_ANGLE = M_PI_2f;
    constexpr float SPEED_MAX_UP = 8.0f;
    constexpr float SPEED_MAX_DOWN = 6.0f;
    constexpr float THRUST_SAFE_LIMIT = 1.0f;
    constexpr float HOVER_THRUST = 0.51f;

    // Actuator: sync with airframe file
    constexpr double ROTOR_MIN = 10.0;
    constexpr double ROTOR_MAX = 1500.0;
    constexpr double ROTOR_RANGE = ROTOR_MAX - ROTOR_MIN;

    constexpr double ELEVON_MIN = -M_PI_4;
    constexpr double ELEVON_MAX = M_PI_4;
    constexpr double ELEVON_RANGE = M_PI_2;

    constexpr double TILT_MIN = 0.0;
    constexpr double TILT_MAX = M_PI_2;
    constexpr double TILT_RANGE = M_PI_2;

    // Safety
    constexpr float RADIUS = 1.2f; // Single sphere drone radius in meter
    constexpr float SAFE_BUFFER = 0.1f; // Tune-able
    constexpr float HAZARD_DISTANCE = RADIUS + SAFE_BUFFER; // Single sphere hazard distance

    // Dynamic hazard distance
    inline float hazard_distance(float radius) {
        return radius + SAFE_BUFFER;
    }

    constexpr float SLEW_TIME = 1.0f; // s, Max time it takes enter max deceleration
    constexpr float LOOKAHEAD_TIME = 1.0f; // s
    constexpr float DECELERATE_MAX = 2.0f; // m/s^2
    constexpr float DECELERATE_SMOOTH = 1.0f; // m/s^2

    // Braking_distance = radius + speed * SLEW_TIME + (speed_sq / (2 * DECELERATE_MAX));
    // About 2.65m at the speed of 1m/s with the current config
    inline float braking_distance(float speed, float radius) {
        return {
            radius +
            speed * SLEW_TIME +
            (speed * speed) / (2 * DECELERATE_MAX)
        };
    };

    // Pushing_distance = radius + speed * SLEW_TIME + (speed_sq / (2 * DECELERATE_SMOOTH));
    // About 6.4m at the speed of 1m/s with the current config
    inline float pushing_distance(float speed, float radius) {
        return {
            radius + 
            speed * LOOKAHEAD_TIME + 
            (speed * speed) / (2 * DECELERATE_SMOOTH)
        };
    };

    // Body box anti clipping, independent with sensor mount point, sensor data have to be body frame transformed
    // Currently unuse, but should be kept
    constexpr float MAX_X = 0.53f + 0.05f;
    constexpr float MIN_X = -0.4f - 0.05f;
    constexpr float MAX_Y = 1.075f + 0.05f;
    constexpr float MIN_Y = -1.075f - 0.05f;
    constexpr float MAX_Z = 0.21f + 0.05f;
    constexpr float MIN_Z = -0.1f -0.05f; 
}

namespace Sensor {
    //: LiDAR 1D down
    constexpr float LIDAR_DOWN_RANGE_MIN = 0.1f;
    constexpr float LIDAR_DOWN_RANGE_MIN_SQ = LIDAR_DOWN_RANGE_MIN * LIDAR_DOWN_RANGE_MIN;
    constexpr float LIDAR_DOWN_RANGE_MAX = 30.0f;
    constexpr float LIDAR_DOWN_RANGE_MAX_SQ = LIDAR_DOWN_RANGE_MAX * LIDAR_DOWN_RANGE_MAX;

    //: LiDAR 2D circle
    constexpr uint8_t LIDAR_2D_SECTOR_NUM = 12;
    constexpr int LIDAR_2D_SIZE = 1440;
    constexpr float LIDAR_2D_RANGE_MIN = 0.1f;
    constexpr float LIDAR_2D_RANGE_MIN_SQ = LIDAR_2D_RANGE_MIN * LIDAR_2D_RANGE_MIN;
    constexpr float LIDAR_2D_RANGE_MAX = 30.0f;
    constexpr float LIDAR_2D_RANGE_MAX_SQ = LIDAR_2D_RANGE_MAX * LIDAR_2D_RANGE_MAX;

    //: Depth Camera
    constexpr float DEPTH_CAM_RANGE = 30.0f;

    //: Voxblox map default
    constexpr float VOXEL_RESOLUTION = 0.5f;
    constexpr size_t VOXEL_PER_SIDE = 8; // new voxel allocate to chunk of block 8x8x8 = 512 voxels, a 2 m^3 cube
    constexpr float VOXEL_TRUNCATION_DISTANCE = VOXEL_RESOLUTION * 4.0f;
    constexpr float VOXEL_MIN_WEIGHT = 1.0f;
    constexpr float VOXEL_MAX_WEIGHT = 100.0f;
    // constexpr float VOXEL_DROPOFF_EPSILON_WEIGHT = 15.0f; // Not been put in Low Spatial config

    //: Acrobatic OA
    constexpr int SPATIAL_BOX_DIM = 12;

}

namespace Physics {
    constexpr float STEP_SIZE = 0.001f;
}

//_                                   FUNCTION
namespace Global {
    void setup_for_simulation(rclcpp::Node *node); // Set up clock sync in simulation
}

