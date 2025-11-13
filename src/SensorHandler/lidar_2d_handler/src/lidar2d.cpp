#include "lidar_2d_handler/lidar2d.hpp"

#define DEBUG 0
#define DEBUG_POINT DEBUG && 1

Lidar2dHandlerNode::Lidar2dHandlerNode() : rclcpp::Node("lidar2d_handler_node") {
    using namespace std::chrono_literals;

    // Create Subscriber
    raw_scan_SUB = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/sensor/lidar_2d/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&Lidar2dHandlerNode::ScanCallback, this, _1)
    );

    vehicle_local_position_SUB = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position",
        rclcpp::SensorDataQoS(),
        std::bind(&Lidar2dHandlerNode::LocalPositionCallback, this, _1)
    );

    // Create Publisher
    obstacle_close_PUB = this->create_publisher<ros2_msgs::msg::Lidar2dObstacle>(CLOSE_TOPIC_DIR, rclcpp::SensorDataQoS());

    obstacle_far_PUB = this->create_publisher<ros2_msgs::msg::Lidar2dObstacle>(FAR_TOPIC_DIR, rclcpp::SensorDataQoS());
    
    // Create Wall timer
    sensor_alive_timer = this->create_wall_timer(
        std::chrono::nanoseconds(HEART_BEAT_CYCLE),
        std::bind(&Lidar2dHandlerNode::SensorHealthCallback, this)
    );

    consumer_close = std::thread(&Lidar2dHandlerNode::ConsumerLoop,
        this,
        std::ref(queue_close), 
        std::ref(obstacle_close_PUB),
        std::string(CLOSE_TOPIC_DIR), 
        "CLOSE");
    consumer_far = std::thread(&Lidar2dHandlerNode::ConsumerLoop, 
        this, 
        std::ref(queue_far),
        std::ref(obstacle_far_PUB), 
        std::string(FAR_TOPIC_DIR),
        "FAR");
}

Lidar2dHandlerNode::~Lidar2dHandlerNode() {
    node_running = false;
    sendPublishSignal(queue_close);
    sendPublishSignal(queue_far);
    if(consumer_close.joinable()) consumer_close.join();
    if(consumer_far.joinable()) consumer_far.join();
}

/**
 * @brief Filter lidar scan into 2 thread: one for close range and the other for long range
 * The result is elements of the array range put into queue on those 2 thread
 */
void Lidar2dHandlerNode::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Lidar Call back timestamp %u",msg->header.stamp.nanosec);
    if(init_sensor_specs) {
        angle_min = msg->angle_min;
        angle_max = msg->angle_max;
        angle_increasement = msg->angle_increment;
        range_min = msg->range_min;
        range_max = msg->range_max;
        lidar_2d_size = static_cast<int>(std::floor((angle_max - angle_min) / angle_increasement)) +1;
        init_sensor_specs = false;
    }

    last_timestamp.store(this->get_clock()->now().nanoseconds());
    
    float current_direction, safe_distance;
    {
        std::lock_guard<std::mutex> lock(mutex_movement);
        current_direction = movement_current.arc;
        safe_distance = movement_current.distance;
    }
    
    #if DEBUG && 0
    int index = 0;
    float angle_index = angle_min;
    while(index < lidar_2d_size) {
        Point point;
        point.arc = angle_index;
        if(0 <= index && index < lidar_2d_size) {
            point.distance = ClampLidar(msg->ranges[index]);
            #if 0
                printf("Enqueue [%.06f] = %.2f\n", point.arc, point.distance);
                raise(SIGTRAP);
            #endif
            if(point.distance == CLEAR) {
                queue_close.enqueue(point);
                queue_far.enqueue(point);
            }
            else {
                if(point.distance < safe_distance) queue_close.enqueue(point);
                else queue_far.enqueue(point);
            }
        }
        angle_index += angle_increasement;
        index++;
    }
    #else
    method_sector.sectorItoratorInit(method_sector.angleToSector(current_direction));
    for(uint i = 0; i < SECTOR_NUM; ++i) {
        uint8_t current_sector = method_sector.sectorItoratorNext();
        const float angle_start = method_sector.getAngleStartSector(current_sector);
        const float angle_end = method_sector.getAngleEndSector(current_sector);

        const int index_start = ceil((angle_start - angle_min) / angle_increasement);
        if(index_start < 0) continue;
        else if(index_start >= lidar_2d_size) continue;

        int index_end = floor((angle_end - angle_min) / angle_increasement);
        if(index_end < 0) continue;
        else if(index_end > lidar_2d_size -1) index_end = lidar_2d_size - 1;

        #if DEBUG && 0
        printf("Sector %d\n", current_sector);
        printf("Start at index %d, end at index %d, number of rays = %d\n", index_start, index_end, index_end - index_start +1);
        raise(SIGTRAP);
        #endif
        int index = 0;
        while(index_start + index <= index_end) {
            Point point;
            point.arc = angle_start + index*angle_increasement;
            // point.arc = round(point.arc * 1e6) / 1e6;
            if(0 <= index && index < lidar_2d_size) {
                point.distance = ClampLidar(msg->ranges[index_start + index]);
                #if DEBUG && 0
                printf("Enqueue [%.06f] = %.2f\n", point.arc, point.distance);
                raise(SIGTRAP);
                #endif
                if(point.distance == CLEAR) {
                    queue_close.enqueue(point);
                    queue_far.enqueue(point);
                }
                else {
                    if(point.distance < safe_distance) queue_close.enqueue(point);
                    else queue_far.enqueue(point);
                }
            }
            index++;
       } 
    }
    #endif

    sendPublishSignal(queue_close);
    sendPublishSignal(queue_far);
}

void Lidar2dHandlerNode::LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    float speed = sqrt(msg->vx * msg->vx + msg->vy * msg->vy + msg->vz * msg->vz);
    {
        std::lock_guard<std::mutex> lock(mutex_movement);
        if(speed > 0.05) movement_current.arc = atan2(msg->vy, msg->vx) - msg->heading;
        else movement_current.arc = 0;
        movement_current.distance = HAZARD_DISTANCE + speed * REACT_TIME + ((speed * speed) / (2 * DECELERATE_MAX));
    }
    RCLCPP_DEBUG(this->get_logger(), "Local Position Call back timestamp %lu: speed= %0.2f; direction= %0.2f; safe distance= %0.2f."
        ,msg->timestamp, speed, movement_current.arc, movement_current.distance
    );
}

void Lidar2dHandlerNode::SensorHealthCallback() {
    auto time_now =  this->get_clock()->now().nanoseconds();
    auto time_last = last_timestamp.load();
    auto time_apart = (time_now >= time_last) ? (time_now - time_last) : (0);
    sensor_alive = time_apart <= HEART_BEAT_CYCLE;
    if(last_timestamp == 0){
        sensor_alive = false;
        RCLCPP_INFO(this->get_logger(), YELLOW "[Lidar_2d_Handler] No init scan yet" RESET);
        return;
    }
    #if !DEBUG
        if(!sensor_alive) RCLCPP_INFO(this->get_logger(), RED "[Lidar_2d_Handler] Lost scan callback" RESET);
    #endif
}

float Lidar2dHandlerNode::ClampLidar(float range) {
    if(range_min <= range && range <= range_max) return range;
    else return CLEAR;
}

bool Lidar2dHandlerNode::checkPublishSignal(const Point& point) {
    if(point.arc == END_TOKEN.arc && point.distance == END_TOKEN.distance) return true;
    return false;
}

void Lidar2dHandlerNode::sendPublishSignal(moodycamel::BlockingConcurrentQueue<Point>& queue) {
    queue.enqueue(END_TOKEN);
}

/**
 * @brief Read rays from queue, handle when to separate contour (to sector), publish topic
 */
void Lidar2dHandlerNode::ConsumerLoop(moodycamel::BlockingConcurrentQueue<Point>& queue,
    rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr& pub,
    const std::string& publish_dir,
    const std::string& label) {
    Obstacle obstacle;
    Contour contour;
    uint8_t sector_now = 0, sector_before = 0;
    #if DEBUG_POINT
    Point last_entry;
    #endif
    bool new_scan = true;
    while(node_running) {
        Point entry = {1, 0, 0, 0};
        queue.wait_dequeue(entry);
        if(!node_running) break;
        #if DEBUG && 0
            printf("Dequeue [%.6f] = %.2f\n", entry.arc, entry.distance);
            if(fabs(entry.arc - 135*DEGREE) < 1e-4) {
                printf(RED "Here entry 135 Degree\n" RESET);
                raise(SIGTRAP);
            }
        #endif
        if(checkPublishSignal(entry)) { // Producer finish!
            if(!contour.empty()) {
                obstacle.addContour(sector_before, contour);
                contour = Contour();
            }
            if(obstacle.getObstaclesNum() > 0) publishData(pub, obstacle, label);
            obstacle = Obstacle(); // New scan
            new_scan = true;
        }
        else {
            if(entry.distance != CLEAR) { // There are contour
                sector_now = obstacle.angleToSector(entry.arc);
                if(!new_scan && sector_before != sector_now) {
                    if(!contour.empty()) {
                        #if DEBUG_POINT
                        printf("Add end point [%.9f (%0.9f)] = %.2f to sector %d\n", 
                            angleInPolar(last_entry.arc) / DEGREE, 
                            last_entry.arc, 
                            last_entry.distance, 
                            sector_before
                        );
                        printf("Obstacle ended with %ld points\n", contour.getContour().size());
                        #endif
                        obstacle.addContour(sector_before, contour);
                        contour = Contour();
                    }
                }
                contour.tryAddPoint(entry);
                #if DEBUG_POINT
                if(sector_before != sector_now) {
                    printf("Add First Point [%.9f (%0.9f)] = %.2f added to sector %d\n", 
                        angleInPolar(entry.arc) / DEGREE, 
                        entry.arc, 
                        entry.distance, 
                        sector_now
                    );
                }
                last_entry = entry;
                #endif
                sector_before = sector_now;
            }
            else { // Obstacle ended
                if(!contour.empty()) {
                    #if DEBUG_POINT
                    printf("Add end point [%.9f (%0.9f)] = %.2f to sector %d\n", 
                            angleInPolar(last_entry.arc) / DEGREE, 
                            last_entry.arc, 
                            last_entry.distance, 
                            sector_before
                        );
                    int cz = contour.getContour().size();
                    if(cz > 1) printf("Obstacle ended in sector %d with %d point\n", sector_before, cz);
                    else printf(YELLOW "Obstacle ended in sector %d with %d point\n" RESET, sector_before, cz);
                    #endif
                    obstacle.addContour(sector_before, contour);
                    contour = Contour();
                }
            }
            new_scan = false;
        }
    }
}

/**
 * @brief Serialize the class sector into individual contour, repersented by pair of 2 float (arc, distance).
 * vector float = [(arc1 distance1 float2 distance2 ... )], each contour end by END_TOKEN (69, 0).
 */
void Lidar2dHandlerNode::publishData(rclcpp::Publisher<ros2_msgs::msg::Lidar2dObstacle>::SharedPtr& pub,
    Obstacle& obstacle,
    const std::string& label) {
    if(obstacle.getObstaclesNum() == 0) return;
    std::vector<ros2_msgs::msg::Lidar2dSector> obstacles = obstacle.obstacleToTopic();
    if(obstacles.empty()) return;

    // Move obstacles to msg and publish
    auto msg = ros2_msgs::msg::Lidar2dObstacle();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";
    msg.obstacles_num = obstacle.getObstaclesNum();
    msg.obstacles = obstacles;
    pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), GREEN "Publish topic %s: %d Obstacle." RESET,label.c_str(), obstacle.getObstaclesNum());
}
