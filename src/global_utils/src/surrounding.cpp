#include "global_utils/surrounding.hpp"

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

// ##################################### Class Contour
Contour::Contour() {
    points.clear();
}

bool Contour::tryAddPoint(Point& point) {
    point.x = point.distance * cosf(point.arc);
    point.y = point.distance * sinf(point.arc);
    
    if(points.size() >= 2 && makeStraightLine(point)) {
        points.pop_back();
    }
    points.push_back(point);
    min_distance = std::min(min_distance, points.back().distance);
    #if DEBUG && 0
    if(points.size() == 1) printf("New obstacle begin\n");
    for(auto point : points) {
        printf("[%.4f] = %.2f\n", point.arc, point.distance);
    }
    printf("-----------------------------\n");
    #endif
    return true;
}

const std::vector<Point>& Contour::getContour() const {
    return this->points;
}

float Contour::getMinDistance() const {
    return min_distance;
}

bool Contour::empty() {
    return points.empty();
}

bool Contour::makeStraightLine(const Point& point) {
    const int cz = points.size();
    if(cz <= 1) return true;
    const float &x1 = points[cz - 2].x;
    const float &y1 = points[cz - 2].y;
    const float &x2 = points.back().x;
    const float &y2 = points.back().y;
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
            points.back().arc, points.back().distance,
            getDistance2Point(points.back(), points[cz - 2]),
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

float Contour::getDistance2Point(const Point& pointA, const Point& pointB) {
    return hypot(pointA.x - pointB.x, pointA.y - pointB.y);
}

// ################################## Class Obstacle

Obstacle::Obstacle() {
    sector[0].sector_angle_start = -SECTOR_ARC/2;
    sector[0].sector_angle_stop = SECTOR_ARC/2;
    for(uint8_t i = 1; i < SECTOR_NUM; i++) {
        sector[i].sector_angle_start = floor(angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC * (i - 1)) * 1e6) / 1e6;
        sector[i].sector_angle_stop = floor(angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC* i) * 1e6) / 1e6;
    }
    sector->min_distance = 0;
}

void Obstacle::topicToObstacle(const std::vector<ros2_msgs::msg::Lidar2dSector>& obstacle) {
    // clear all sectors
    for (auto& s : sector)
        s.contours.clear();

    for (const auto& msg_sector : obstacle) {
        if (msg_sector.sector_index >= SECTOR_NUM)
            continue;  // invalid index

        auto& internal_sector = sector[msg_sector.sector_index];
        internal_sector.sector_angle_start = msg_sector.start_angle;
        internal_sector.sector_angle_stop = msg_sector.end_angle;
        internal_sector.min_distance = msg_sector.min_distance;

        for (const auto& msg_contour : msg_sector.contours) {
            Contour contour;
            contour.setMinDistance(msg_contour.min_distance);
            for (const auto& msg_point : msg_contour.points) {
                Point point;
                point.arc = msg_point.arc;
                point.distance = msg_point.distance;
                point.x = msg_point.x;
                point.y = msg_point.y;
                contour.addPoint(point);
            }
            internal_sector.contours.push_back(contour);
        }
    }
}

std::vector<ros2_msgs::msg::Lidar2dSector> Obstacle::obstacleToTopic() {
    std::vector<ros2_msgs::msg::Lidar2dSector> obstacle;

    for(uint8_t sector_index = 0; sector_index < SECTOR_NUM; sector_index++) {
        auto s = ros2_msgs::msg::Lidar2dSector();
        s.sector_index = sector_index;
        s.start_angle = sector[sector_index].sector_angle_start;
        s.end_angle = sector[sector_index].sector_angle_stop;
        s.min_distance = sector[sector_index].min_distance;

        for(Contour contour : sector[sector_index].contours) {
            auto c = ros2_msgs::msg::Lidar2dContour();
            c.min_distance = contour.getMinDistance();
            for(Point point : contour.getContour()) {
                auto p = ros2_msgs::msg::Lidar2dPoint();
                p.arc = point.arc;
                p.distance = point.distance;
                p.x = point.x;
                p.y = point.y;
                c.points.push_back(p);
            }
            s.contours.push_back(c);
        }
        obstacle.push_back(s);
    }
    return obstacle;
}

void Obstacle::addContour(const uint8_t& sector_index, Contour& new_contour) {
    if(new_contour.empty()) return;
    sector[sector_index].contours.push_back(new_contour);
    obstacles_num++;
    sector[sector_index].min_distance = std::min(
        sector[sector_index].min_distance, 
        sector[sector_index].contours.back().getMinDistance()
    );
    min_distance = std::min(
        min_distance,
        sector[sector_index].min_distance
    );
}

uint8_t Obstacle::angleToSector(float angle) {
    angle = angleInPolar(angle + SECTOR_ARC/2) + 0.0004f;
    float result = floor(angle / SECTOR_ARC);
    return static_cast<uint8_t>(result) % SECTOR_NUM;
}

uint16_t Obstacle::getObstaclesNum() {
    return obstacles_num;
}

float Obstacle::getMinDistance() {
    return min_distance;
}

float Obstacle::getAngleStartSector(const uint8_t& index) {
    return sector[index].sector_angle_start;
}

float Obstacle::getAngleEndSector(const uint8_t& index) {
    return sector[index].sector_angle_stop;
}

float Obstacle::getMinDistanceSector(const uint8_t& index) {
    return sector[index].min_distance;
}

void Obstacle::sectorItoratorInit(const uint8_t& start_index = 0) {
    sector_iterator = start_index;
    sector_der = 0;
    sector_iter_init = true;
}

uint8_t Obstacle::sectorItoratorNext() {
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

std::vector<Contour> Obstacle::getObstacles(const uint8_t& index) {
    return this->sector[index].contours;
}

