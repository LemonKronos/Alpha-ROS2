#pragma once

#include <memory>
#include <shared_mutex>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"

namespace alpha_brain {
    struct MapState {
        // The Memory Lock
        std::shared_mutex mutex;

        // The 3D Voxel Memory
        std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer = nullptr;

        // The Configuration (Compile-Time Defaults)
        voxblox::TsdfIntegratorBase::Config config;

        // You can add a constructor here to force compile-time initialization
        // of specific Voxblox settings!
        MapState() {
            // These compile directly into the binary
            //! Dynamic these value
            config.default_truncation_distance = Sensor::VOXEL_TRUNCATION_DISTANCE; 
            config.max_weight = Sensor::VOXEL_MAX_WEIGHT;
            config.voxel_carving_enabled = true;
        }
    };

    inline MapState global_map;

} // namespace alpha_brain