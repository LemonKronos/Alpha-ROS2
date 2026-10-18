#pragma once

#include <memory>
#include <shared_mutex>
#include <Eigen/Dense>

#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/core/esdf_map.h"
#include "voxblox/integrator/tsdf_integrator.h"
#include "voxblox/integrator/esdf_integrator.h"

#include "global_utils/system_config.hpp"

namespace spatial_container {

//! Only Low Spatial Node shall init and config this
class MapManager {
public:
    MapManager() = default;
    ~MapManager() = default;

    MapManager(const MapManager&) = delete;
    MapManager& operator=(const MapManager&) = delete;

    // The shared lock for all nodes to use manually if needed
    mutable std::shared_mutex mutex;

    // Intergrator config
    voxblox::TsdfIntegratorBase::Config tsdf_dense_config;
    voxblox::TsdfIntegratorBase::Config tsdf_sparse_config;
    voxblox::EsdfIntegrator::Config esdf_config;

    // The 3D Voxel Memory Layers and Map Wrapper
    std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer = nullptr;
    std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer = nullptr;
    std::shared_ptr<voxblox::EsdfMap> esdf_map = nullptr;
    std::unique_ptr<voxblox::EsdfIntegrator> esdf_integrator = nullptr; 

    void initialize(
        float voxel_resolution,
        voxblox::TsdfIntegratorBase::Config tsdf_dense_config,
        voxblox::TsdfIntegratorBase::Config tsdf_sparse_config,
        voxblox::EsdfIntegrator::Config esdf_config
    ) {
        std::unique_lock<std::shared_mutex> lock(mutex);
        this->tsdf_dense_config = tsdf_dense_config;
        this->tsdf_sparse_config = tsdf_sparse_config;
        this->tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);
        this->esdf_config = esdf_config;
        this->esdf_layer = std::make_shared<voxblox::Layer<voxblox::EsdfVoxel>>(voxel_resolution, Sensor::VOXEL_PER_SIDE);
        
        // EsdfMap provides the interpolation and gradient queries over the raw layer
        this->esdf_map = std::make_shared<voxblox::EsdfMap>(this->esdf_layer);
        
        this->esdf_integrator = std::make_unique<voxblox::EsdfIntegrator>(
            this->esdf_config, 
            this->tsdf_layer.get(), 
            this->esdf_layer.get()
        );
    }

    void clear() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        if (tsdf_layer) tsdf_layer->removeAllBlocks();
        if (esdf_layer) esdf_layer->removeAllBlocks();
    }

    void updateEsdf() {
        std::unique_lock<std::shared_mutex> lock(mutex);
        if (esdf_integrator) {
            esdf_integrator->updateFromTsdfLayer(true); // true = clear updated flags
        }
    }

    // --- Clean Interface for Reactive OA ---
    
    bool getDistance(const Eigen::Vector3f& pos, float& distance) const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        if (!esdf_map) return false;
        
        double dist_d = 0.0;
        bool obs = esdf_map->getDistanceAtPosition(pos.cast<double>(), &dist_d);
        distance = static_cast<float>(dist_d);
        return obs;
    }

    bool getDistanceAndGradient(const Eigen::Vector3f &pos, float &distance, Eigen::Vector3f &gradient) const {
        std::shared_lock<std::shared_mutex> lock(mutex);
        if (!esdf_map) return false;

        double dist_d = 0.0; 
        Eigen::Vector3d grad_d;
        bool obs = esdf_map->getDistanceAndGradientAtPosition(pos.cast<double>(), &dist_d, &grad_d);
        distance = static_cast<float>(dist_d);
        gradient = grad_d.cast<float>();
        return obs;
    }
};

// Global instance in spatial_container shared memory
inline MapManager global_map;

} // namespace spatial_container