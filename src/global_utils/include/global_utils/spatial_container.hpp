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
        if (!esdf_integrator) return;

        esdf_integrator->updateFromTsdfLayer(true);

        #if ALLOW_DEBUG && 0
        // after esdf_integrator->updateFromTsdfLayer(true);
        size_t n_pos_def = 0, n_neg_def = 0, n_other = 0, n_in_queue = 0, n_fixed = 0, n_halluc = 0;
        float dmin = 1e9f, dmax = -1e9f;
        int printed_occ = 0, printed_near = 0;

        voxblox::BlockIndexList idx;
        tsdf_layer->getAllAllocatedBlocks(&idx);
        for (const auto& i : idx) {
            const auto& tb = tsdf_layer->getBlockByIndex(i);
            auto eb = esdf_layer->getBlockPtrByIndex(i);
            if (!eb) continue;
            for (size_t v = 0; v < tb.num_voxels(); ++v) {
                const auto& t = tb.getVoxelByLinearIndex(v);
                const auto& e = eb->getVoxelByLinearIndex(v);
                if (t.weight <= 1e-6f || !e.observed) continue;

                dmin = std::min(dmin, e.distance); dmax = std::max(dmax, e.distance);
                if (e.in_queue) ++n_in_queue;
                if (e.fixed) ++n_fixed;
                if (e.hallucinated) ++n_halluc;
                if (std::abs(e.distance - esdf_config.default_distance_m) < 1e-3f) ++n_pos_def;
                else if (std::abs(e.distance + esdf_config.default_distance_m) < 1e-3f) ++n_neg_def;
                else ++n_other;

                if (t.distance <= esdf_config.min_distance_m && printed_occ < 5) {
                    fprintf(stderr, "[occ ] tsdf d=%.3f w=%.1f | esdf d=%.3f fixed=%d inq=%d\n",
                            t.distance, t.weight, e.distance, e.fixed, e.in_queue);
                    ++printed_occ;
                } else if (t.distance > 0.0f && t.distance < 0.5f && printed_near < 5) {
                    fprintf(stderr, "[near] tsdf d=%.3f w=%.1f | esdf d=%.3f fixed=%d inq=%d\n",
                            t.distance, t.weight, e.distance, e.fixed, e.in_queue);
                    ++printed_near;
                }
            }
        }
        fprintf(stderr, "[esdf] range=[%.2f, %.2f] +def=%zu -def=%zu other=%zu inqueue=%zu fixed=%zu halluc=%zu\n",
                dmin, dmax, n_pos_def, n_neg_def, n_other, n_in_queue, n_fixed, n_halluc);
        #endif
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