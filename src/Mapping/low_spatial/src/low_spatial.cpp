#include "low_spatial/low_spatial.hpp"

spatial_container::LowSpatialNode::LowSpatialNode(const rclcpp::NodeOptions& options) :
    Node("low_spatial", options)
{
        Global::setup_for_simulation(this);

        //_ ROS2 params
        this->declare_parameters("", std::map<std::string, rclcpp::ParameterValue>{
            //: Base
            {"voxel_resolution", rclcpp::ParameterValue(-1.0)}, // use as kill pill
            {"truncation_distance", rclcpp::ParameterValue(Sensor::VOXEL_TRUNCATION_DISTANCE)},
            {"max_weight", rclcpp::ParameterValue(Sensor::VOXEL_MAX_WEIGHT)},
            {"voxel_carving_enabled", rclcpp::ParameterValue(true)},
            {"integrator_threads", rclcpp::ParameterValue(4)},

            //: Dense
            {"dense_use_weight_dropoff", rclcpp::ParameterValue(true)},
            {"dense_use_compensate", rclcpp::ParameterValue(false)},
            {"dense_compensation_factor", rclcpp::ParameterValue(10.0f)},
            {"dense_max_ray_length", rclcpp::ParameterValue(20.0)},

            //: Sparse
            {"sparse_max_ray_length", rclcpp::ParameterValue(20.0)},
            {"sparse_compensation_factor", rclcpp::ParameterValue(5.0)},

            //: ESDF
            {"esdf_full_euclidean", rclcpp::ParameterValue(true)},
            {"esdf_min_distance", rclcpp::ParameterValue(0.2)},
            {"esdf_max_distance", rclcpp::ParameterValue(8.0)},
            {"esdf_default_distance", rclcpp::ParameterValue(10.0)}
        });

        // Check load params
        float voxel_resolution = static_cast<float>(this->get_parameter("voxel_resolution").as_double());
        if(voxel_resolution < 0) { // kill pill
            RCLCPP_WARN(this->get_logger(), YELLOW "Config not loaded, use default" RESET);
            voxel_resolution = Sensor::VOXEL_RESOLUTION;
        }

        //_ Init variables
        //: TSDF Base
        voxblox::TsdfIntegratorBase::Config tsdf_dense_config;
        tsdf_dense_config.default_truncation_distance = static_cast<float>(this->get_parameter("truncation_distance").as_double());
        tsdf_dense_config.max_weight = static_cast<float>(this->get_parameter("max_weight").as_double());
        tsdf_dense_config.voxel_carving_enabled = this->get_parameter("voxel_carving_enabled").as_bool();
        tsdf_dense_config.integrator_threads = std::max<int>(1, static_cast<int>(this->get_parameter("integrator_threads").as_int()));
        
        //: TSDF Dense
        tsdf_dense_config.use_weight_dropoff = this->get_parameter("dense_use_weight_dropoff").as_bool();
        tsdf_dense_config.use_sparsity_compensation_factor = this->get_parameter("dense_use_compensate").as_bool();
        tsdf_dense_config.sparsity_compensation_factor = static_cast<float>(this->get_parameter("dense_compensation_factor").as_double());
        tsdf_dense_config.max_ray_length_m = static_cast<float>(this->get_parameter("dense_max_ray_length").as_double());

        //: TSDF Sparse - modify from the Dense
        voxblox::TsdfIntegratorBase::Config tsdf_sparse_config = tsdf_dense_config;
        tsdf_sparse_config.use_const_weight = true;
        tsdf_sparse_config.use_sparsity_compensation_factor = true;
        tsdf_sparse_config.sparsity_compensation_factor = static_cast<float>(this->get_parameter("sparse_compensation_factor").as_double());
        tsdf_sparse_config.max_ray_length_m = static_cast<float>(this->get_parameter("sparse_max_ray_length").as_double());

        //: ESDF
        voxblox::EsdfIntegrator::Config esdf_config;
        esdf_config.full_euclidean_distance = this->get_parameter("esdf_full_euclidean").as_bool();
        esdf_config.min_distance_m = this->get_parameter("esdf_min_distance").as_double();
        esdf_config.max_distance_m = this->get_parameter("esdf_max_distance").as_double();
        esdf_config.default_distance_m = this->get_parameter("esdf_default_distance").as_double();

        if(esdf_config.min_distance_m <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), RED "esdf_min_distance must be > 0 (got %.3f), the ESDF would never propagate. Using 0.2" RESET, esdf_config.min_distance_m);
            esdf_config.min_distance_m = 0.2;
        }
        if(esdf_config.default_distance_m <= esdf_config.max_distance_m) {
            esdf_config.default_distance_m = esdf_config.max_distance_m + 1.0;
            RCLCPP_WARN(this->get_logger(), YELLOW "esdf_default_distance must be > esdf_max_distance, using %.2f" RESET, esdf_config.default_distance_m);
        }

        //: Init Voxblox map
        global_map.initialize(voxel_resolution, tsdf_dense_config, tsdf_sparse_config, esdf_config);

        RCLCPP_INFO(this->get_logger(), GREEN "Voxblox map ready (ESDF: full_euclidean=%d min=%.2f max=%.2f default=%.2f)" RESET,
            static_cast<int>(esdf_config.full_euclidean_distance),
            esdf_config.min_distance_m, esdf_config.max_distance_m, esdf_config.default_distance_m);

        #if TIME_ANALYSE
        analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger());
        #endif

        //_ Timers
        esdf_update_TIME = this->create_timer(
            std::chrono::nanoseconds(Clock::LOOP_CYCLE_SLOW_NANOSEC),
            std::bind(&spatial_container::LowSpatialNode::EsdfUpdateCallback, this)
        );
}

spatial_container::LowSpatialNode::~LowSpatialNode() {
    #if TIME_ANALYSE
    analyzer->printSummary();
    #endif
}

void spatial_container::LowSpatialNode::EsdfUpdateCallback() {
    #if TIME_ANALYSE
    analyzer->start_segment("esdf update");
    #endif

    spatial_container::global_map.updateEsdf();

    #if TIME_ANALYSE
    analyzer->stop_segment("esdf update");
    #endif
}
