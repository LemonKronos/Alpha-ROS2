#pragma once

#include "Eigen/Dense"

// FOAM - multi-sphere safe bubble
struct FoamSphere {
    Eigen::Vector3f pos;
    float radius;
};
