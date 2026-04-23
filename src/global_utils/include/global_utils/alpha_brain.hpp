#pragma once

#include <memory>
#include <shared_mutex>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace alpha_brain {
    // The Read-Write lock to prevent core dumps from thread collisions.
    // - DepthCam must use: std::unique_lock<std::shared_mutex> lock(global_map_mutex);
    // - AcrobaticOA must use: std::shared_lock<std::shared_mutex> lock(global_map_mutex);
    inline std::shared_mutex global_map_mutex;

    // The zero-copy shared map pointer. 
    // - Mapping component calls make_shared to create it.
    // - DepthCam component writes to it.
    // - AcrobaticOA component reads from it.
    inline std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> global_tsdf_layer = nullptr;

} // namespace alpha_brain