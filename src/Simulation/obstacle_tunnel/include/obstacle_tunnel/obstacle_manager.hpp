#ifndef OBSTACLE_MANAGER_HPP_
#define OBSTACLE_MANAGER_HPP_

#include <string>
#include <vector>
#include <cmath>

struct TunnelConfig {
    float drone_size;      
    float width;           
    float height;          
    float block_size;      
    float slice_depth;     
};

struct Point3D {
    float x, y, z;
};

class ObstacleManager {
public:
    ObstacleManager();

    // Returns SDF for a SINGLE slice (Walls + Obstacles)
    std::string generate_slice_xml(int slice_index);
    
    // Returns the safe Teleport Z height (Floor + buffer)
    float get_floor_z() const;
    
    float get_slice_depth() const { return config_.slice_depth; }

private:
    TunnelConfig config_;
    unsigned int master_seed_;

    std::string generate_walls_sdf(float x_center);
    std::string generate_obstacles_sdf(int slice_index, float x_center);
    
    // Deterministic path generator
    Point3D get_tunnel_waypoint(int slice_index);

    std::string create_box_link(std::string name, float x, float y, float z, float dx, float dy, float dz, bool is_obstacle, float r, float g, float b);
};

#endif // OBSTACLE_MANAGER_HPP_