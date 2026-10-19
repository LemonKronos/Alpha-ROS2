#pragma once

#include "Eigen/Dense"

// FOAM - multi-sphere safe bubble
struct FoamSphere {
    Eigen::Vector3f pos;
    float radius;
};

// Cache ESDF data to use when read locked
struct SphereCache {
    bool valid = false;
    int age_ticks = 0;
    float distance = 0.0f;
    Eigen::Vector3f gradient_world = Eigen::Vector3f::Zero();
    Eigen::Vector3f center_at_read = Eigen::Vector3f::Zero();
};