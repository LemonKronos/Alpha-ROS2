#include "obstacle_tunnel/obstacle_manager.hpp"
#include "global_utils/system_config.hpp"
#include <sstream>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm> 

// Helper for Rainbow Colors
struct Color { float r, g, b; };

Color get_sector_color(int index) {
    int hue_step = std::abs(index) % 7; 
    // Muted Rainbow Palette (High value, lower saturation)
    switch(hue_step) {
        case 0: return {0.7f, 0.3f, 0.3f}; // Muted Red
        case 1: return {0.7f, 0.5f, 0.3f}; // Muted Orange
        case 2: return {0.7f, 0.7f, 0.3f}; // Muted Yellow
        case 3: return {0.3f, 0.7f, 0.3f}; // Muted Green
        case 4: return {0.3f, 0.7f, 0.7f}; // Muted Cyan
        case 5: return {0.3f, 0.4f, 0.8f}; // Muted Blue
        case 6: return {0.6f, 0.3f, 0.8f}; // Muted Purple
    }
    return {0.5f, 0.5f, 0.5f};
}

ObstacleManager::ObstacleManager() {
    config_.drone_size = Drone::RADIUS * 2.0f; 
    
    // --- TUNING ---
    config_.width = 5.0f * config_.drone_size;
    config_.height = 5.0f * config_.drone_size;
    
    // Detailed resolution
    config_.block_size = config_.drone_size * 0.25f; 
    
    // Sector Depth
    config_.slice_depth = 6.0f * config_.drone_size; 

    // Generate a true random seed for this session
    std::random_device rd;
    master_seed_ = rd(); 
}

float ObstacleManager::get_floor_z() const {
    return 0.2f; 
}

Point3D ObstacleManager::get_tunnel_waypoint(int slice_index) {
    Point3D p;
    p.x = slice_index * config_.slice_depth;
    
    // Path Wandering
    p.y = (config_.width * 0.3f) * sin(slice_index * 0.5f) + 
          (config_.width * 0.1f) * cos(slice_index * 1.1f);
          
    p.z = (config_.height * 0.5f) + 
          (config_.height * 0.2f) * cos(slice_index * 0.4f);
          
    return p;
}

std::string ObstacleManager::generate_slice_xml(int slice_index) {
    float slice_x = slice_index * config_.slice_depth; 
    
    std::stringstream sdf;
    sdf << "<?xml version='1.0'?>\n";
    sdf << "<sdf version='1.6'>\n";
    sdf << "<model name='slice_" << slice_index << "'>\n";
    sdf << "  <static>true</static>\n";
    
    sdf << generate_walls_sdf(slice_x);
    sdf << generate_obstacles_sdf(slice_index, slice_x);

    sdf << "</model>\n";
    sdf << "</sdf>";
    return sdf.str();
}

std::string ObstacleManager::generate_walls_sdf(float x_center) {
    std::stringstream ss;
    float len = config_.slice_depth;
    float w = config_.width;
    float h = config_.height;
    float t = 0.1f; 

    // Tunnel Frame - Simple color
    ss << create_box_link("floor", x_center, 0, -t/2, len, w, t, false, 0.4, 0.8, 0.0);
    ss << create_box_link("ceil", x_center, 0, h + t/2, len, w, t, false, 0.0, 0.9, 0.9);
    ss << create_box_link("wall_L", x_center, w/2 + t/2, h/2, len, t, h, false, 0.2, 0.2, 0.2);
    ss << create_box_link("wall_R", x_center, -w/2 - t/2, h/2, len, t, h, false, 0.2, 0.2, 0.2);

    return ss.str();
}

std::string ObstacleManager::generate_obstacles_sdf(int slice_index, float x_center) {
    // Keep start area clear
    if (std::abs(slice_index) < 1) return ""; 

    std::stringstream ss;
    // std::mt19937 rng(slice_index * 9999); // Old - fix slides
    std::mt19937 rng(master_seed_ + (slice_index * 9999)); // New - random each run

    std::uniform_real_distribution<> dist(0.0, 1.0);

    // --- TUNING KNOBS ---
    float HOLE_SCALE = 1.5f; // 2.0x larger holes (Easier)
    int DECOY_MIN = 4;       // Minimum extra holes
    int DECOY_VAR = 3;       // Random variance (4 to 7 holes)

    int rows = (int)(config_.height / config_.block_size);
    int cols = (int)(config_.width / config_.block_size);

    // 1. Initialize 2D Grid (True = Solid Wall)
    std::vector<std::vector<bool>> grid(rows, std::vector<bool>(cols, true));

    // 2. Determine "Golden Path" Hole
    Point3D pt = get_tunnel_waypoint(slice_index);
    
    // Use the slice index to determine the hole type/rotation
    // 0: Horizontal Rect (Normal)
    // 1: Vertical Rect (Sideways/Roll)
    // 2: Small Box (Precision)
    int type = std::abs(slice_index) % 3;
    float hw, hh; // Hole Width, Hole Height

    if (type == 0) { 
        hw = config_.drone_size * 1.0f; 
        hh = config_.drone_size * 0.6f;
    } else if (type == 1) {
        hw = config_.drone_size * 0.5f; // Requires sideways flight
        hh = config_.drone_size * 1.2f;
    } else {
        hw = config_.drone_size * 0.7f;
        hh = config_.drone_size * 0.7f;
    }

    // APPLY SCALING
    hw *= HOLE_SCALE;
    hh *= HOLE_SCALE;

    // 3. Carve Main Hole
    int cy = (int)((pt.y + config_.width/2) / config_.block_size);
    int cz = (int)(pt.z / config_.block_size);
    int rw = (int)((hw/2) / config_.block_size);
    int rh = (int)((hh/2) / config_.block_size);

    for(int r = cz - rh; r <= cz + rh; r++) {
        for(int c = cy - rw; c <= cy + rw; c++) {
            if(r >= 0 && r < rows && c >= 0 && c < cols) grid[r][c] = false;
        }
    }

    // 4. Carve "Decoy" Holes (Multiple Open)
    int decoys = DECOY_MIN + (int)(dist(rng) * DECOY_VAR); 
    
    for(int k=0; k<decoys; k++) {
        int dy = (int)(dist(rng) * cols);
        int dz = (int)(dist(rng) * rows);
        
        // Randomize decoy size, but also scale it so they are flyable
        // Base size is 1.5 drone block sizes * HOLE_SCALE
        float decoy_radius_base = (config_.drone_size / config_.block_size) * 0.5f;
        int dr = (int)(decoy_radius_base * (0.8f + dist(rng)) * HOLE_SCALE); 
        
        // Ensure at least 1 block radius
        if (dr < 1) dr = 1;

        for(int r = dz - dr; r <= dz + dr; r++) {
            for(int c = dy - dr; c <= dy + dr; c++) {
                if(r >= 0 && r < rows && c >= 0 && c < cols) grid[r][c] = false;
            }
        }
    }

    // 5. GREEDY MESHING
    // Get Color for this entire Slide
    Color sec_col = get_sector_color(slice_index);

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (grid[r][c]) {
                // Start a strip
                int width_count = 1;
                while ((c + width_count) < cols && grid[r][c + width_count]) {
                    grid[r][c + width_count] = false; // Mark as consumed
                    width_count++;
                }

                // Create Link
                float b_sz = config_.block_size;
                
                // Position X: Center of slice
                float bx = x_center;
                
                // Position Y: Center of the strip
                float start_y = -(config_.width/2) + (c * b_sz);
                float by = start_y + (width_count * b_sz / 2.0f);
                
                // Position Z: Center of row
                float bz = (r * b_sz) + (b_sz/2.0f);

                std::string name = "w_" + std::to_string(r) + "_" + std::to_string(c);
                
                ss << create_box_link(name, bx, by, bz, 
                                      b_sz, // Thickness (1 block deep)
                                      width_count * b_sz, // Width (Merged)
                                      b_sz, // Height (1 block)
                                      true, sec_col.r, sec_col.g, sec_col.b);
            }
        }
    }

    return ss.str();
}

std::string ObstacleManager::create_box_link(std::string name, float x, float y, float z, float dx, float dy, float dz, bool is_obstacle, float r, float g, float b) {
    if (dx < 0.001f) dx = 0.001f;
    if (dy < 0.001f) dy = 0.001f;
    if (dz < 0.001f) dz = 0.001f;

    std::stringstream ss;
    ss << "  <link name='" << name << "'>\n";
    ss << "    <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
    ss << "    <visual name='v'>\n";
    ss << "      <geometry><box><size>" << dx << " " << dy << " " << dz << "</size></box></geometry>\n";
    
    if (is_obstacle) {
        // High opacity for obstacles
        ss << "      <material><ambient>" << r << " " << g << " " << b << " 1</ambient>";
        ss << "<diffuse>" << r << " " << g << " " << b << " 1</diffuse></material>\n";
    } else {
        // Lower opacity for walls/floor
        ss << "      <material><ambient>0.1 0.1 0.1 0.5</ambient><diffuse>0.1 0.1 0.1 0.5</diffuse></material>\n";
    }
    ss << "    </visual>\n";
    ss << "    <collision name='c'>\n";
    ss << "      <geometry><box><size>" << dx << " " << dy << " " << dz << "</size></box></geometry>\n";
    ss << "    </collision>\n";
    ss << "  </link>\n";
    return ss.str();
}