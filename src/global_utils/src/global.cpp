#include "global_utils/global.hpp"

#define DEBUG 0
#if DEBUG
#include <iostream>
#include <csignal>
#endif

/* ########################################## Function*/
float angleInWrapped(float angle) {
    while(angle < -M_PI) angle += 2*M_PI;
    while(angle > M_PI) angle -= 2*M_PI;
    return angle;
}

float angleInPolar(float angle) {
    while(angle < 0) angle += 2*M_PI;
    while(angle > 2*M_PI) angle -=2*M_PI;
    return angle;
}

template <typename T>
T linearMap(const T& input, 
    const T& in_min, const T& in_max, 
    const T& out_min, const T& out_max
) {
    if(in_min == in_max) return (out_max - out_min)/2;
    else if(in_min < in_max) {
        T ratio = (input - in_min) / (in_max - in_min);
        if(ratio < 0) ratio = 0;
        if(ratio > 1) ratio = 1;
        return out_min + ratio*(out_max - out_min);
    }
    else {
        T ratio = (input - in_max) / (in_min - in_max);
        if(ratio < 0) ratio = 0;
        if(ratio > 1) ratio = 1;
        return out_min + (1 - ratio)*(out_max - out_min);
    }
}

template <typename T>
T duoLinearMap(
    const T& inA, const T& inA_min, const T& inA_max,
    const T& inB, const T& inB_min, const T& inB_max,
    const T& out_min, const T& out_max  
) {
    auto normalize = [](T x, T minv, T maxv) {
        if (minv == maxv) return T(0);  // Avoid division-by-zero

        // If inverted range: swap interpolation direction
        if (minv < maxv) {
            return (x - minv) / (maxv - minv);  // normal
        } else {
            return (minv - x) / (minv - maxv);  // inverted
        }
    };

    T scaleA = normalize(inA, inA_min, inA_max);
    T scaleB = normalize(inB, inB_min, inB_max);

    // Clamp to [0, 1]
    if (scaleA < 0) scaleA = 0; else if (scaleA > 1) scaleA = 1;
    if (scaleB < 0) scaleB = 0; else if (scaleB > 1) scaleB = 1;

    T ratio = scaleA * scaleB;

#if DEBUG
    printf("scaleA = %f, scaleB = %f, ratio = %f\n",
           (double)scaleA, (double)scaleB, (double)ratio);
#endif

    return out_min + ratio * (out_max - out_min);
}


template <typename T>
T expoMap(const T& input, const T& in_min, const T& in_max, const T& out_min, const T& out_max, const T& sensitivity) {
    if (in_max <= in_min)
        return out_min;

    T ratio = (input - in_min) / (in_max - in_min);
    if(ratio < 0) ratio = 0;
    if(ratio > 1) ratio = 1;

    T expo_ratio = pow(ratio, sensitivity);
    return out_min + expo_ratio * (out_max - out_min);
}

// ##################################### Class ObstacleArc
ObstacleArc::ObstacleArc() {
    contour.clear();
}

bool ObstacleArc::tryAddPoint(Point& point) {
    point.x = point.distance * cosf(point.arc);
    point.y = point.distance * sinf(point.arc);
    
    if(contour.size() >= 2 && makeStraightLine(point)) {
        contour.pop_back();
    }
    contour.push_back(point);
    min_distance = std::min(min_distance, contour.back().distance);
    #if DEBUG && 0
    if(contour.size() == 1) printf("New obstacle begin\n");
    for(auto point : contour) {
        printf("[%.4f] = %.2f\n", point.arc, point.distance);
    }
    printf("-----------------------------\n");
    #endif
    return true;
}

const std::vector<Point>& ObstacleArc::getContour() const {
    return this->contour;
}

float ObstacleArc::getMinDistance() const {
    return min_distance;
}

bool ObstacleArc::empty() {
    return contour.empty();
}

bool ObstacleArc::makeStraightLine(const Point& point) {
    const int cz = contour.size();
    if(cz <= 1) return true;
    const float &x1 = contour[cz - 2].x;
    const float &y1 = contour[cz - 2].y;
    const float &x2 = contour.back().x;
    const float &y2 = contour.back().y;
    const float &x3 = point.x;
    const float &y3 = point.y;
    const float derivate = fabs((y3 - y2)*(x2 - x1) - (y2 - y1)*(x3 - x2));

    const float threshold = linearMap(
        point.distance, 0.01f, 30.0f,
        0.017455f, 0.176327f
    );

    if(derivate < threshold) {
        return true;
    }
    else {
        #if DEBUG && 1
        printf("New point [%f] = %0.2f, distance last 2 = %0.2f, threshold = %f = %f degree\n",
            contour.back().arc, contour.back().distance,
            getDistance2Point(contour.back(), contour[cz - 2]),
            threshold,
            threshold / DEGREE
            ); 
        printf(RED "No straight line with derivated slop = %f = %f degree\n" RESET,
            derivate,
            atanf(derivate) / DEGREE
        );
        raise(SIGTRAP);
        #endif
        return false;
    }
    return false;
}

float ObstacleArc::getDistance2Point(const Point& pointA, const Point& pointB) {
    return hypot(pointA.x - pointB.x, pointA.y - pointB.y);
}

// ####################################### Class ObstacleMap

bool ObstacleMap::tryAddPoint(const Point& point ) {
    if(contour.size() <= 1) {
        contour.push_back(point);
    }
    else if(checkDistanceAddPoint(point)) {
        if(makeStraightLine(point)) contour.pop_back();
        contour.push_back(point);
        return true;
    }
    return false;
}

const std::vector<Point>& ObstacleMap::getContour() const {
    return this->contour;
}

bool ObstacleMap::makeStraightLine(const Point& point) {
    float x[2], y[2];
    int cz = contour.size();
    for(auto i = 0; i < 2; i++) {
        x[i] = contour[cz -1 -i].x;
        y[i] = contour[cz -1 -i].y;
    }
    float distance = sqrtf((point.x - x[0])*(point.x - x[0]) + (point.y - y[0])*(point.y - y[0]));
    float thresshold = tolerance_factor / distance;
    if(
        std::abs((point.y - y[0]) / (point.x - x[0]) - (point.y - y[1]) / (point.x - x[1])) 
        <= thresshold
    ) return true;
    else return false;
}

bool ObstacleMap::checkDistanceAddPoint(const Point& point) {
    if(getDistance2Point(contour.back(), point) > SIGNIFICANT_DISTANCE)
        return true;
    return false;
}

float ObstacleMap::getDistance2Point(const Point& pointA, const Point& pointB) {
    return sqrtf((pointA.x - pointB.x) + (pointA.y - pointB.y));
}

// ################################## Class Sector

Sector::Sector() {
    data[0].sector_angle_start = -SECTOR_ARC/2;
    data[0].sector_angle_stop = SECTOR_ARC/2;
    for(uint8_t i = 1; i < SECTOR_NUM; i++) {
        data[i].sector_angle_start = floor(angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC * (i - 1)) * 1e6) / 1e6;
        data[i].sector_angle_stop = floor(angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC* i) * 1e6) / 1e6;
    }
    data->min_distance = 0;
}

void Sector::addObstacleArc(const uint8_t& sector_index, ObstacleArc& new_obstacle) {
    if(new_obstacle.empty()) return;
    data[sector_index].obstacle_arc.push_back(new_obstacle);
    obstacles_num++;
    data[sector_index].min_distance = std::min(
        data[sector_index].min_distance, 
        data[sector_index].obstacle_arc.back().getMinDistance()
    );
}

uint8_t Sector::angleToSector(float angle) {
    angle = angleInPolar(angle + SECTOR_ARC/2) + 0.0004f;
    float result = floor(angle / SECTOR_ARC);
    return static_cast<uint8_t>(result) % SECTOR_NUM;
}

uint16_t Sector::getObstaclesNum() {
    return obstacles_num;
}

float Sector::getAngleStartSector(const uint8_t& index) {
    return data[index].sector_angle_start;
}

float Sector::getAngleEndSector(const uint8_t& index) {
    return data[index].sector_angle_stop;
}

float Sector::getMinDistanceSector(const uint8_t& index) {
    return data[index].min_distance;
}

void Sector::sectorItoratorInit(const uint8_t& start_index = 0) {
    sector_iterator = start_index;
    sector_der = 0;
    sector_iter_init = true;
}

uint8_t Sector::sectorItoratorNext() {
    if(sector_iter_init) {
        sector_iter_init = false;
        return sector_iterator;
    }
    int8_t temp = static_cast<int8_t>(sector_iterator);
    if(sector_der <= 0) {
        sector_der = -sector_der + 1;
    }
    else sector_der = -sector_der;
    temp = (temp + sector_der) % SECTOR_NUM;
    while(temp < 0) temp += SECTOR_NUM;
    return static_cast<uint8_t>(temp);
}

std::vector<ObstacleArc> Sector::getObstacles(const uint8_t& index) {
    return this->data[index].obstacle_arc;
}

// #################################### Class Map

Map::Map() {

}

void Map::convertSectorToMap(Sector& sector) {
    for(uint8_t i = 0; i < SECTOR_NUM; i++) {
        auto obstacle_arc_list = sector.getObstacles(i);
        for(auto obstacle_arc : obstacle_arc_list) {
            ObstacleMap obstacle_map;
            auto contour_arc = obstacle_arc.getContour();
            for(auto point_arc : contour_arc) {
                Point point_map = convertPointArcToMap(sector.origin, point_arc);
                obstacle_map.tryAddPoint(point_map);
            }
            obstacle_map_list.push_back(obstacle_map);
        }
    }
}

void Map::addObstacle(const ObstacleMap& obstacle) {
    obstacle_map_list.push_back(obstacle);
}

Point Map::convertPointArcToMap(const Point& origin, const Point& point_arc) {
    Point point_map;
    point_map.x = point_arc.distance * cosf(point_arc.arc) + origin.x;
    point_map.y = point_arc.distance * sinf(point_arc.arc) + origin.y;
    return point_map;
}
