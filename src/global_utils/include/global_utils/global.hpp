#pragma once

#include <vector>
#include <cstdint>
#include <float.h>
#include <cmath>

/* ######################################## Constant */
constexpr uint8_t SECTOR_NUM = 12;
constexpr float SECTOR_ARC = 2 * M_PI / SECTOR_NUM;
constexpr float DEGREE = 0.017453292f;

// Need real data or tinkering
constexpr float SELF_RADIUS = 0.23f; // radius in meter
constexpr float UNCERTAINTY = 0.01f;
constexpr float HAZARD_DISTANCE = SELF_RADIUS + UNCERTAINTY;
constexpr float REACT_TIME = 0.03f; // ms
constexpr float DECELERATE_MAX = 4.0f; // m/s^2
constexpr float SAFE_BUFFER = 0.01f;

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

class ObstacleArc {
public:
    ObstacleArc();
    bool tryAddPoint(Point& point); // Clustering here
    const std::vector<Point>& getContour() const;
    float getMinDistance() const;
    bool empty();

private:
    std::vector<Point> contour;
    float min_distance = FLT_MAX;
    bool makeStraightLine(const Point& point); // Tolerance scale with distance
    float getDistance2Point(const Point& pointA, const Point& pointB);
};

class ObstacleMap {
public:
    bool tryAddPoint(const Point& point);
    const std::vector<Point>& getContour() const;

private:
    std::vector<Point> contour;
    float tolerance_factor = 5; // I choose for now, linear, reverse scale
    static constexpr float SIGNIFICANT_DISTANCE = 0.01; // meter
    bool makeStraightLine(const Point& point); // Tolerance scale with 1/distance
    bool checkDistanceAddPoint(const Point& point);
    float getDistance2Point(const Point& pointA, const Point& pointB);
};
/**
 * @brief Sector arc stored in Counter Clock-wise Wrapped frame, e.i, from +PI to -PI looking Clock-wise
 * 
 * Sector 0 start from -SECTOR_ARC/2 to SECTOR_ARC/2
 * 
 * @note The sector names are also iterated in Couter Clock-wise
 */
class Sector {
public:
    struct Data {
        float sector_angle_start;
        float sector_angle_stop;
        float min_distance = FLT_MAX;

        std::vector<ObstacleArc> obstacle_arc;
    };
    Point origin = {0, 0, 0, 0};

    Sector();
    void addObstacleArc(const uint8_t& sector_index, ObstacleArc& new_obstacle);
    uint8_t angleToSector(float angle);
    void sectorItoratorInit(const uint8_t& start_index);
    uint8_t sectorItoratorNext();

    // get methods
    uint16_t getObstaclesNum();
    float getAngleStartSector(const uint8_t& index);
    float getAngleEndSector(const uint8_t& index);
    float getMinDistanceSector(const uint8_t& index);
    std::vector<ObstacleArc> getObstacles(const uint8_t& index);

private:
    Data data[SECTOR_NUM];
    uint16_t obstacles_num = 0;
    uint8_t sector_iterator;

    // Helper
    bool sector_iter_init = true;
    int8_t sector_der = 0;
};

class Map { // This will get refactor to oblivion after finish ReactiveOA, depend by local_mapping node
public:
    Map();
    void convertSectorToMap(Sector& sector);
    void addObstacle(const ObstacleMap& obstacle);
private:
    std::vector<ObstacleMap> obstacle_map_list;
    Point convertPointArcToMap(const Point& origin, const Point& point_arc);
};