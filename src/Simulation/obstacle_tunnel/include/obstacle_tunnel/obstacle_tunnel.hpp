#ifndef OBSTACLE_TUNNEL_NODE_HPP_
#define OBSTACLE_TUNNEL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <set>
#include <memory>

#include "global_utils/system_config.hpp"

// Native Gazebo Interfaces
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <ros_gz_interfaces/srv/delete_entity.hpp>
#include <ros_gz_interfaces/srv/control_world.hpp> 
#include <ros_gz_interfaces/srv/set_entity_pose.hpp> 

// Custom & PX4 Messages
#include "alpha_msgs/msg/fuse_perception.hpp" 
#include "alpha_msgs/msg/record_control.hpp"
#include "px4_msgs/msg/vehicle_status.hpp" 

#include "obstacle_tunnel/obstacle_manager.hpp"

// FSM State Definition
enum class TunnelState {
    RUNNING,            
    WAITING_FOR_DISARM, 
    RESETTING           
};

class ObstacleTunnelNode : public rclcpp::Node {
public:
    ObstacleTunnelNode();
    ~ObstacleTunnelNode(); // Destructor for cleanup

private:
    // --- Callbacks ---
    void perception_callback(const alpha_msgs::msg::FusePerception::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    // --- Actions ---
    void manage_slices(float drone_x);
    void execute_soft_reset();
    
    void spawn_slice(int index);
    void delete_slice(int index);
    void cleanup_all_slices(); // Helper for destructor

    // --- Subscriptions ---
    rclcpp::Subscription<alpha_msgs::msg::FusePerception>::SharedPtr perception_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    // --- Publishers ---
    rclcpp::Publisher<alpha_msgs::msg::RecordControl>::SharedPtr record_pub_;

    // --- Service Clients ---
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr control_world_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;

    // --- Managers & State ---
    std::unique_ptr<ObstacleManager> manager_;
    
    TunnelState current_state_ = TunnelState::RUNNING;
    uint8_t current_arming_state_ = 0; 
    
    float slice_depth_;
    float render_buffer_ = 30.0f; 
    float current_drone_x_ = 0.0f;
    Name::Dynamic::DRONE drone_name;
    
    int bearable_crash_count_ = 0;
    
    std::set<int> active_slice_indices_;
};

#endif // OBSTACLE_TUNNEL_NODE_HPP_