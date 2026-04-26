#include <rclcpp/rclcpp.hpp>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include "voxblox/integrator/tsdf_integrator.h"

#include "global_utils/system_config.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/alpha_brain.hpp"


namespace alpha_brain {

class LowSpatialNode : public rclcpp::Node {
public:
    LowSpatialNode(const rclcpp::NodeOptions& options);
    ~LowSpatialNode();

private:

};

} // namespace alpha_brain
