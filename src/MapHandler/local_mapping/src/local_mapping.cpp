#include "local_mapping/local_mapping.hpp"

#define DEBUG_MODE true
#define VISUALIZER true

LocalMappingNode::LocalMappingNode():rclcpp::Node("local_mapping_node") {
    using namespace std::chrono_literals;

    // Create publisher


    // Create subscriber
    obstacle_arc_close_SUB = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/close",
        10,
        std::bind(&LocalMappingNode::obstacleCloseCallback, this, _1)
    );

    obstacle_arc_far_SUB = this->create_subscription<ros2_msgs::msg::Lidar2dObstacle>(
        "/on_drone/sensor/scan/lidar2d/far",
        10,
        std::bind(&LocalMappingNode::obstacleFarCallback, this, _1)
    );

    // Create wall timer

    if(VISUALIZER) {
        Visualizer vis(this);
        vis_pointer = &vis;
    }
}

LocalMappingNode::~LocalMappingNode() {}

void LocalMappingNode::obstacleCloseCallback(const ros2_msgs::msg::Lidar2dObstacle:: SharedPtr msg) {
    instance_close_map = Map();
    convertLidar2dObstacleTopicToMap(msg, instance_close_map);
    if(VISUALIZER) {
        vis_pointer->PublishLidar2dContour(instance_close_map, "close");
    }
}

void LocalMappingNode::obstacleFarCallback(const ros2_msgs::msg::Lidar2dObstacle:: SharedPtr msg) {
    instance_far_map = Map();
    convertLidar2dObstacleTopicToMap(msg, instance_far_map);
    if(VISUALIZER) {
        vis_pointer->PublishLidar2dContour(instance_far_map, "far");
    }
}

void LocalMappingNode::convertLidar2dObstacleTopicToMap(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg, Map& map) {
    std::vector<float> lidar_2d_array = msg->pointarray; // 
    // int obstacles_num = msg->obstacles_num;
    size_t array_index = 0;
    ObstacleMap obstacle;
    while(array_index < lidar_2d_array.size()) {
        if(array_index % 2 != 0) {
            array_index++;
            continue;
        }
        if(lidar_2d_array[array_index] != FLT_MAX) {
            PointMap point;
            point.x = lidar_2d_array[array_index +1] * cos(lidar_2d_array[array_index]);
            point.y = lidar_2d_array[array_index +1] * sin(lidar_2d_array[array_index]);
            obstacle.tryAddPoint(point);
        }
        else {
            if(!obstacle.getContour().empty()) {
                map.addObstacle(obstacle);
                obstacle = ObstacleMap();
            }
        }
        array_index +=2;
    }
}