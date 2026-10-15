#pragma once

#include <memory>
#include <shared_mutex>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"

namespace alpha_brain {

//! Only Low Spatial Node shall init and config this
class MapManager {
public:
    MapManager() = default;
    ~MapManager() = default;

    MapManager(const MapManager&) = delete;
    MapManager& operator=(const MapManager&) = delete;

    // The shared lock for all nodes to use manually
    std::shared_mutex mutex;

    // Base config
    voxblox::TsdfIntegratorBase::Config config;

    // The 3D Voxel Memory Layers
    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer = nullptr;
    std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer = nullptr;

    void initialize(float voxel_resolution, voxblox::TsdfIntegratorBase::Config config) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        this->config = config;
        this->tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);
        this->esdf_layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);
    }

    void clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        if (tsdf_layer) tsdf_layer->removeAllBlocks();
        if (esdf_layer) esdf_layer->removeAllBlocks();
    }
};

// Global instance in alpha_brain shared memory
inline MapManager global_map;

} // namespace alpha_brain