#include "depth_cam/depth_cam.hpp"

alpha_brain::DepthCamNode::DepthCamNode(const rclcpp::NodeOptions & options) :
    Node("octo_map_node", options)
    {

    Global::setup_for_simulation(this);

    // Frame transform
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Create subcriber
    fuse_perception_SUB = this->create_subscription<ros2_msgs::msg::FusePerception>(
        Topic::FUSE_PERCEPTION,
        rclcpp::SensorDataQoS(),
        std::bind(&alpha_brain::DepthCamNode::FusePerceptionCallback, this, _1)
    );

    // Init variables
    hazard_point_thread = std::make_unique<HazardPointThread>(this, 3);
    world_update_thread = std::make_unique<WorldUpdateThread>(this, 3);
    front_processing_thread = std::make_unique<ProcessingThread>(
        "Front",
        this,
        Topic::DEPTH_CAM_FRONT_PL,
        tf_buffer,
        hazard_point_thread->getQueue(),
        world_update_thread->getStatus(),
        world_update_thread->getQueue()
    );

    left_processing_thread = std::make_unique<ProcessingThread>(
        "Left",
        this,
        Topic::DEPTH_CAM_LEFT_PL,
        tf_buffer,
        hazard_point_thread->getQueue(),
        world_update_thread->getStatus(),
        world_update_thread->getQueue()
    );

    right_processing_thread = std::make_unique<ProcessingThread>(
        "Right", 
        this,
        Topic::DEPTH_CAM_RIGHT_PL,
        tf_buffer,
        hazard_point_thread->getQueue(),
        world_update_thread->getStatus(),
        world_update_thread->getQueue()
    );
}

alpha_brain::DepthCamNode::~DepthCamNode() {
    // Nothing here
}

void alpha_brain::DepthCamNode::FusePerceptionCallback(const ros2_msgs::msg::FusePerception::SharedPtr msg) {
    float hazard_distance_sq = msg->hazard_distance_sq;
    front_processing_thread->updateSafeBubble(hazard_distance_sq);
    left_processing_thread->updateSafeBubble(hazard_distance_sq);
    right_processing_thread->updateSafeBubble(hazard_distance_sq);
}