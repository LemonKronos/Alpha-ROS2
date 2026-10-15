#include "depth_cam/depth_cam.hpp"

alpha_brain::DepthCamNode::DepthCamNode(const rclcpp::NodeOptions & options) :
    Node("depth_cam_node", options)
    {

    Global::setup_for_simulation(this);

    // Frame transform
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Init variables
    analyzer = std::make_unique<time_utils::TimeAnalyzer>(this->get_logger(), 300);

    world_update_thread = std::make_unique<WorldUpdateThread>(this, analyzer.get());
    
    front_processing_thread = std::make_unique<ProcessingThread>(
        "Front",
        this,
        Topic::DEPTH_CAM_FRONT_PL,
        tf_buffer,
        world_update_thread->getQueue(),
        analyzer.get()
    );

    left_processing_thread = std::make_unique<ProcessingThread>(
        "Left",
        this,
        Topic::DEPTH_CAM_LEFT_PL,
        tf_buffer,
        world_update_thread->getQueue(),
        analyzer.get()
    );

    right_processing_thread = std::make_unique<ProcessingThread>(
        "Right", 
        this,
        Topic::DEPTH_CAM_RIGHT_PL,
        tf_buffer,
        world_update_thread->getQueue(),
        analyzer.get()
    );
}

alpha_brain::DepthCamNode::~DepthCamNode() {
#if TIME_ANALYSE
        analyzer->printSummary();
#endif
}