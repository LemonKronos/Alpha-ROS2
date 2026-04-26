#pragma once

#include <memory>
#include <shared_mutex>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"

namespace alpha_brain {

    // Low Spatial Node shall init and config this
    struct MapState {
        std::shared_mutex mutex;

        voxblox::TsdfIntegratorBase::Config config;

        // The 3D Voxel Memory
        std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer = nullptr;

    };

    // Inline in alpha_brain container shared memory
    inline MapState global_map;

} // namespace alpha_brain