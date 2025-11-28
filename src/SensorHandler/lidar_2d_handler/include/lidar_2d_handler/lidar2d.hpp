#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "global_utils/surrounding.hpp"
#include "global_utils/utils.hpp"
#include "global_utils/system_config.hpp"
#include "global_utils/blockingconcurrentqueue.h" // Moody Camel MPMC Queue with wait blocking
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ros2_msgs/msg/lidar2d_obstacle.hpp"

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <atomic>
#include <cmath>

using std::placeholders::_1;

constexpr uint64_t HEART_BEAT_CYCLE = 1e9;

class Lidar2dHandlerNode : public rclcpp::Node {
public:
    Lidar2dHandlerNode();
    ~Lidar2dHandlerNode();

private:
    // Publishers
    rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr obstacle_close_PUB;
    rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr obstacle_far_PUB;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_SUB;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_SUB;

    // Variables
    moodycamel::BlockingConcurrentQueue<Point> queue_close;
    moodycamel::BlockingConcurrentQueue<Point> queue_far;
    std::thread consumer_close;
    std::thread consumer_far;
    std::atomic<bool> node_running{true};
    bool sensor_alive = true;
    std::atomic<rcl_time_point_value_t> last_timestamp{0};
    Point movement_current = {0, 0.5, 0, 0}; // forward, safe bubble 0.5 meter
    std::mutex mutex_movement;
    Obstacle method_sector;
    
    // Sensor specs
    bool init_sensor_specs = true;
    float angle_min, angle_max, angle_increasement;
    float range_min, range_max;
    int lidar_2d_size;
    
    // Timers
    rclcpp::TimerBase::SharedPtr sensor_alive_timer;

    // Methods
    void ConsumerLoop(moodycamel::BlockingConcurrentQueue<Point>& queue,
        rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr& pub,
        const std::string& publish_dir,
        const std::string& label);
    float ClampLidar(float range);
    void sendPublishSignal(moodycamel::BlockingConcurrentQueue<Point>& queue);
    bool checkPublishSignal(const Point& point);
    void publishData(rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr& pub,
        Obstacle& obstacle,
        const std::string& label
    ); // If Obstacle empty then no publish

    // Callbacks
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void SensorHealthCallback();

};
