#include "obstacle_tunnel/obstacle_tunnel.hpp"
#include "global_utils/system_config.hpp" 
#include <cmath> 
#include <thread>
#include <iostream> 

using std::placeholders::_1;
using namespace std::chrono_literals;

ObstacleTunnelNode::ObstacleTunnelNode() : Node("obstacle_tunnel") {
    // Parameters
    this->declare_parameter("drone_name", "alpha_minus_2_0");
    drone_name_ = this->get_parameter("drone_name").as_string();

    manager_ = std::make_unique<ObstacleManager>();
    slice_depth_ = manager_->get_slice_depth();

    std::string world_name = "obstacle_tunnel"; 
    
    // --- GZ Clients ---
    spawn_client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>("/world/" + world_name + "/create");
    delete_client_ = this->create_client<ros_gz_interfaces::srv::DeleteEntity>("/world/" + world_name + "/remove");
    control_world_client_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>("/world/" + world_name + "/control");
    set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/" + world_name + "/set_pose");

    // --- Subscriptions ---
    perception_sub_ = this->create_subscription<ros2_msgs::msg::FusePerception>(
        FUSE_PERCEPTION_TOPIC, rclcpp::SensorDataQoS(), 
        std::bind(&ObstacleTunnelNode::perception_callback, this, _1));

    // PX4 Status (Best Effort)
    rclcpp::QoS px4_qos(10);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        px4_qos,
        std::bind(&ObstacleTunnelNode::vehicle_status_callback, this, _1)
    );

    // --- Publishers ---
    record_pub_ = this->create_publisher<ros2_msgs::msg::RecordControl>(
        LOGGER_RECORD_TOPIC, 10);

    // --- Shutdown Hook ---
    rclcpp::on_shutdown([this]() {
        std::cout << "\n[ObstacleTunnel] Shutdown signal received. Cleaning up tunnel slices...\n";

        // 1. Clean up tunnel (Fire and forget delete requests)
        this->cleanup_all_slices();

        // 2. Small wait to ensure delete commands leave the network buffer
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "[ObstacleTunnel] Cleanup requested. Bye!\n";
    });

    RCLCPP_INFO(this->get_logger(), "Tunnel Node Active. Slice Depth: %.2f", slice_depth_);
    
    // Initial Connection Check
    if(control_world_client_->wait_for_service(std::chrono::seconds(5))) {
        // Initial spawn - Safe zone
        spawn_slice(0);
        spawn_slice(1);
        spawn_slice(-1);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Bridge (ControlWorld) not connected!");
    }
}

ObstacleTunnelNode::~ObstacleTunnelNode() {
    // Destructor is handled by on_shutdown mostly.
}

void ObstacleTunnelNode::cleanup_all_slices() {
    for (int idx : active_slice_indices_) {
        auto req = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
        req->entity.name = "slice_" + std::to_string(idx);
        req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
        // Check if client is valid (might be null if constructor failed midway)
        if(delete_client_) delete_client_->async_send_request(req);
    }
    active_slice_indices_.clear();
}

void ObstacleTunnelNode::perception_callback(const ros2_msgs::msg::FusePerception::SharedPtr msg) {
    if (current_state_ != TunnelState::RUNNING) return;

    // --- FIX: NaN Protection ---
    // If fusion dies or PX4 stops, we receive NaN positions. 
    // Trying to calculate slices with NaN results in undefined indices, causing a wipe.
    if (!std::isfinite(msg->position[0])) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "X=nan detected (Fusion Lost). Preserving tunnel state.");
        return;
    }

    current_drone_x_ = msg->position[0]; 
    manage_slices(current_drone_x_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "X=%.1f | Hits: %d | Contact: %d", 
        current_drone_x_, bearable_crash_count_, msg->bearable_contact);

    // Collision Logic
    bool trigger_reset_req = false;
    if (msg->critical_contact) {
        RCLCPP_WARN(this->get_logger(), "CRITICAL HIT! Requesting Disarm...");
        trigger_reset_req = true;
    } else if (msg->bearable_contact) {
        bearable_crash_count_++;
        if (bearable_crash_count_ >= 10) {
            RCLCPP_WARN(this->get_logger(), "Too many hits! Requesting Disarm...");
            trigger_reset_req = true;
        }
    }

    if (trigger_reset_req) {
        current_state_ = TunnelState::WAITING_FOR_DISARM;

        // Stop Recording
        auto rec_msg = ros2_msgs::msg::RecordControl();
        rec_msg.record = false;
        rec_msg.pause = false; 
        record_pub_->publish(rec_msg);
    }
}

void ObstacleTunnelNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    current_arming_state_ = msg->arming_state;

    // FSM Logic: Waiting for Disarm -> Disarmed (1) -> Execute Soft Reset
    if (current_state_ == TunnelState::WAITING_FOR_DISARM && current_arming_state_ == 1) {
        RCLCPP_INFO(this->get_logger(), "Drone Disarmed. Executing Soft Reset.");
        execute_soft_reset();
    }
}

void ObstacleTunnelNode::execute_soft_reset() {
    current_state_ = TunnelState::RESETTING;
    RCLCPP_WARN(this->get_logger(), "--- EXECUTING SOFT RESET ---");

    // 1. Pause Simulation
    auto pause_req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    pause_req->world_control.pause = true;
    control_world_client_->async_send_request(pause_req);

    // 2. Teleport Drone (NO DELETING SLICES)
    if (set_pose_client_->service_is_ready()) {
        auto pose_req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        pose_req->entity.name = drone_name_;
        pose_req->entity.type = 2; // MODEL
        
        // --- SAFE X CALCULATION (THE SEAM) ---
        // Slice N is from [N*depth - depth/2] to [N*depth + depth/2]
        // The SEAM (connection) between current slice and next is at (N + 0.5) * depth
        float current_slice_idx = std::floor(current_drone_x_ / slice_depth_);
        float safe_x = (current_slice_idx + 0.5f) * slice_depth_;
        
        pose_req->pose.position.x = safe_x;
        pose_req->pose.position.y = 0.0;
        pose_req->pose.position.z = manager_->get_floor_z(); // Floor level
        pose_req->pose.orientation.w = 1.0;
        
        set_pose_client_->async_send_request(pose_req);
        RCLCPP_INFO(this->get_logger(), "Teleporting %s to Sector Seam X: %.2f...", drone_name_.c_str(), safe_x);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Teleport Service Unavailable!");
    }

    // 3. Reset Counters
    bearable_crash_count_ = 0;
    
    // Update internal X so slice management doesn't glitch on resume
    current_drone_x_ = std::floor(current_drone_x_ / slice_depth_) * slice_depth_;
    
    // 4. Return to RUNNING (Idle state until Control Node unpauses)
    current_state_ = TunnelState::RUNNING;
    RCLCPP_INFO(this->get_logger(), "Soft Reset Complete. Waiting for Control Node.");
}

void ObstacleTunnelNode::manage_slices(float drone_x) {
    int current_idx = static_cast<int>(std::floor(drone_x / slice_depth_));
    int buffer_count = static_cast<int>(std::ceil(render_buffer_ / slice_depth_));
    int start_idx = current_idx - buffer_count;
    int end_idx = current_idx + buffer_count;

    for (int i = start_idx; i <= end_idx; i++) {
        if (active_slice_indices_.find(i) == active_slice_indices_.end()) {
            spawn_slice(i);
        }
    }

    auto it = active_slice_indices_.begin();
    while (it != active_slice_indices_.end()) {
        int idx = *it;
        if (idx < start_idx || idx > end_idx) {
            delete_slice(idx);
            it = active_slice_indices_.erase(it);
        } else {
            ++it;
        }
    }
}

void ObstacleTunnelNode::spawn_slice(int index) {
    if (active_slice_indices_.count(index) > 0) return;

    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    req->entity_factory.name = "slice_" + std::to_string(index);
    req->entity_factory.sdf = manager_->generate_slice_xml(index); 
    req->entity_factory.allow_renaming = false;
    req->entity_factory.relative_to = "world";
    req->entity_factory.pose.position.x = 0;
    req->entity_factory.pose.position.y = 0;
    req->entity_factory.pose.position.z = 0;
    req->entity_factory.pose.orientation.w = 1.0;

    spawn_client_->async_send_request(req);
    active_slice_indices_.insert(index);
}

void ObstacleTunnelNode::delete_slice(int index) {
    auto req = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
    req->entity.name = "slice_" + std::to_string(index);
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    delete_client_->async_send_request(req);
}