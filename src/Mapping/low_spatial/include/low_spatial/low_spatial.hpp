#include <rclcpp/rclcpp.hpp>

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/spatial_container.hpp"

#ifndef ALLOW_DEBUG
    #define ALLOW_DEBUG 0
#endif

//_ Local define
#define DEBUG (ALLOW_DEBUG & 1)
#define FLOW (ALLOW_DEBUG & 0)
#define TIME_ANALYSE (ALLOW_DEBUG & 0)

namespace spatial_container {

class LowSpatialNode : public rclcpp::Node {
public:
    LowSpatialNode(const rclcpp::NodeOptions& options);
    ~LowSpatialNode();

private:
    // Timer
    rclcpp::TimerBase::SharedPtr esdf_update_TIME;

    // Callbacks
    void EsdfUpdateCallback();

    #if TIME_ANALYSE
    std::unique_ptr<time_utils::TimeAnalyzer> analyzer;
    #endif

};

} // namespace spatial_container
