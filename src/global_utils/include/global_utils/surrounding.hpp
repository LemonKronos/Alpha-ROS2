#pragma once

#include "ros2_msgs/msg/lidar2d_obstacle.hpp"
#include "global_utils/system_config.hpp"

#include <vector>
#include <cstdint>
#include <float.h>
#include <cmath>

/* ######################################## Constant */
constexpr uint8_t SECTOR_NUM = 12;
constexpr float SECTOR_ARC = 2 * M_PI / SECTOR_NUM;

/* ########################################## Function*/
float angleInWrapped(float angle);
float angleInPolar(float angle);

template <typename T>
T linearMap(const T& input, const T& in_min, const T& in_max, const T& out_min, const T& out_max);

// Linear mapping for 2 value, can be invertedly scale
template <typename T>
T duoLinearMap(
    const T& inA, const T& inA_min, const T& inA_max,
    const T& inB, const T& inB_min, const T& inB_max,
    const T& out_min, const T& out_max  
);

template <typename T>
T expoMap(const T& input, const T& in_min, const T& in_max, const T& out_min, const T& out_max, const T& sensitivity);

/* ########################################## Classes*/

struct Point {
    float arc = 0, distance = 0;
    float x = 0, y = 0;
};
constexpr float OVER = 69; // arc over 2Pi
constexpr float CLEAR = 0; // distance clear
constexpr Point END_TOKEN = {OVER, CLEAR, 0, 0};

class Contour {
public:
    Contour();
    bool tryAddPoint(Point& point); // Clustering here
    void addPoint(Point& point) { points.push_back(point); }
    const std::vector<Point>& getContour() const;
    float getMinDistance() const;
    bool empty();

private:
    std::vector<Point> points;
    float min_distance = FLT_MAX;
    void setMinDistance(const float& distance) { min_distance = distance; }
    bool makeStraightLine(const Point& point); // Tolerance scale with distance
    float getDistance2Point(const Point& pointA, const Point& pointB);
    
    friend class Obstacle;
};

/**
 * @brief Sector stored in Counter Clock-wise Wrapped frame, e.i, from +PI to -PI looking Clock-wise
 * Sector 0 start from -SECTOR_ARC/2 to SECTOR_ARC/2
 * @note The sector names are also iterated in Couter Clock-wise
 */
class Obstacle {
public:
    struct Sector {
        float sector_angle_start;
        float sector_angle_stop;
        float min_distance = FLT_MAX;

        std::vector<Contour> contours;
    };
    Point origin = {0, 0, 0, 0};

    Obstacle();
    float safe_distance = FLT_MAX;
    void topicToObstacle(const ros2_msgs::msg::Lidar2dObstacle::SharedPtr msg);
    ros2_msgs::msg::Lidar2dObstacle obstacleToTopic();
    void addContour(const uint8_t& sector_index, Contour& new_contour);
    uint8_t angleToSector(float angle);
    void sectorItoratorInit(const uint8_t& start_index);
    uint8_t sectorItoratorNext();

    // get methods
    uint16_t getObstaclesNum();
    float getMinDistance();
    float getAngleStartSector(const uint8_t& index);
    float getAngleEndSector(const uint8_t& index);
    float getMinDistanceSector(const uint8_t& index);
    std::vector<Contour> getContours(const uint8_t& index);

private:
    Sector sector[SECTOR_NUM];
    uint16_t obstacles_num = 0;
    float min_distance = FLT_MAX;
    uint8_t sector_iterator;

    // Helper
    bool sector_iter_init = true;
    int8_t sector_der = 0;
};
