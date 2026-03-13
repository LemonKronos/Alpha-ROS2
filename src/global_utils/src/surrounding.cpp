#include "global_utils/surrounding.hpp"

#define DEBUG 0
#if DEBUG
#include <iostream>
#include <csignal>
#endif

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
    const float x1 = points[cz - 2].x;
    const float y1 = points[cz - 2].y;
    const float x2 = points.back().x;
    const float y2 = points.back().y;
    const float x3 = point.x;
    const float y3 = point.y;
    const float derivate = fabs((y3 - y2)*(x2 - x1) - (y2 - y1)*(x3 - x2));

    const float threshold = math_utils::linearMap(
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

void Contour::RDPOptimize(float base_epsilon, float growth_factor) {
    if (points.size() < 3) return;

    std::stack<std::pair<size_t, size_t>> stack;
    std::vector<bool> keep_point(points.size(), false);

    keep_point[0] = true;
    keep_point[points.size() - 1] = true;
    stack.push({0, points.size() - 1});

    while (!stack.empty()) {
        std::pair<size_t, size_t> range = stack.top();
        stack.pop();

        size_t start_index = range.first;
        size_t end_index = range.second;

        // --- DYNAMIC EPSILON CALCULATION ---
        // We use the distance of the start point of this segment.
        // Since 'point.distance' is already cached, this is free!
        float segment_dist = points[start_index].distance;
        
        // Example: Base 0.03m + (0.01 * distance)
        // At 1m: epsilon = 0.04m
        // At 10m: epsilon = 0.13m
        // At 30m: epsilon = 0.33m (33cm tolerance)
        float local_epsilon = base_epsilon + (segment_dist * growth_factor);
        
        // Square it for the fast math check
        float epsilon_sq = local_epsilon * local_epsilon;
        // -----------------------------------

        float max_dist_sq = 0.0f;
        size_t furthest_point_index = 0;

        // Cache line deltas
        const Point& p_start = points[start_index];
        const Point& p_end = points[end_index];
        
        float line_dx = p_end.x - p_start.x;
        float line_dy = p_end.y - p_start.y;
        float line_len_sq = line_dx * line_dx + line_dy * line_dy;

        // Iterate points between start and end
        for (size_t i = start_index + 1; i < end_index; ++i) {
            float dist_sq;

            // Handle case where start and end are same point (avoid div by 0)
            if (line_len_sq < 1e-6f) {
                 float dx = points[i].x - p_start.x;
                 float dy = points[i].y - p_start.y;
                 dist_sq = dx * dx + dy * dy;
            } else {
                // Perpendicular Distance Squared Formula:
                // Area = |(y2-y1)x0 - (x2-x1)y0 + x2y1 - y2x1|
                // Dist^2 = Area^2 / Length^2
                float area = (p_end.y - p_start.y) * points[i].x 
                           - (p_end.x - p_start.x) * points[i].y 
                           + p_end.x * p_start.y 
                           - p_end.y * p_start.x;
                dist_sq = (area * area) / line_len_sq;
            }

            if (dist_sq > max_dist_sq) {
                max_dist_sq = dist_sq;
                furthest_point_index = i;
            }
        }

        // If the furthest point is outside our tolerance, split and recurse
        if (max_dist_sq > epsilon_sq) {
            keep_point[furthest_point_index] = true;
            stack.push({start_index, furthest_point_index});
            stack.push({furthest_point_index, end_index});
        }
    }

    // Reconstruct
    std::vector<Point> new_points;
    new_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (keep_point[i]) new_points.push_back(points[i]);
    }
    points = std::move(new_points);
}

// ################################## Class Obstacle

Obstacle::Obstacle() {
    sector[0].sector_angle_start = -SECTOR_ARC/2;
    sector[0].sector_angle_stop = SECTOR_ARC/2;
    for(uint8_t i = 1; i < SECTOR_NUM; i++) {
        sector[i].sector_angle_start = floor(math_utils::angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC * (i - 1)) * 1e6) / 1e6;
        sector[i].sector_angle_stop = floor(math_utils::angleInWrapped(SECTOR_ARC/2 + SECTOR_ARC* i) * 1e6) / 1e6;
    }
}

void Obstacle::topicToObstacle(const alpha_msgs::msg::Lidar2dObstacle::SharedPtr msg) {
    // clear all sectors
    for (auto& s : sector)
        s.contours.clear();

    obstacles_num = msg->obstacles_num;
    min_distance = msg->min_distance;
    safe_distance = msg->safe_distance;

    const auto& obstacle = msg->obstacles;
    for (const auto& msg_sector : obstacle) {
        if (msg_sector.sector_index >= SECTOR_NUM)
            continue;  // invalid index

        auto& internal_sector = sector[msg_sector.sector_index];
        internal_sector.min_distance = msg_sector.min_distance;

        for (const auto& msg_contour : msg_sector.contours) {
            Contour contour;
            // Not contain contour min_distance to reduce data size
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

alpha_msgs::msg::Lidar2dObstacle Obstacle::obstacleToTopic() {
    auto msg = alpha_msgs::msg::Lidar2dObstacle();
    msg.obstacles_num = this->obstacles_num;
    msg.min_distance = this->min_distance;
    msg.safe_distance = this->safe_distance;

    std::vector<alpha_msgs::msg::Lidar2dSector> obstacles;
    for(uint8_t sector_index = 0; sector_index < SECTOR_NUM; sector_index++) {
        if(sector[sector_index].contours.empty()) continue;

        auto s = alpha_msgs::msg::Lidar2dSector();
        s.sector_index = sector_index;
        s.min_distance = sector[sector_index].min_distance;

        for(const Contour& contour : sector[sector_index].contours) {
            auto c = alpha_msgs::msg::Lidar2dContour();
            // Not keep contour min_distance to reduce data size
            for(const Point& point : contour.getContour()) {
                auto p = alpha_msgs::msg::Lidar2dPoint();
                p.arc = point.arc;
                p.distance = point.distance;
                p.x = point.x;
                p.y = point.y;
                c.points.push_back(p);
            }
            s.contours.push_back(c);
        }
        obstacles.push_back(s);
    }
    msg.obstacles = obstacles;
    return msg;
}

void Obstacle::addContour(const uint8_t& sector_index, Contour& new_contour) {
    if(new_contour.empty()) return;
    new_contour.RDPOptimize();
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
    angle = math_utils::angleInPolar(angle + SECTOR_ARC/2) + 0.0004f;
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

std::vector<Contour> Obstacle::getContours(const uint8_t& index) {
    return this->sector[index].contours;
}

