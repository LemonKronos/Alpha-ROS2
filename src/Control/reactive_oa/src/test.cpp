#include "reactive_oa/reactive_oa.hpp"
#define DEBUG 1

class RadialVfhScanner {
public:
    RadialVfhScanner(int t_row, int t_col) 
        : target_row(t_row), target_col(t_col) {
        advance(); // Setup the first valid index immediately
    }

    // Returns the current 1D array index, or -1 if the entire map has been searched
    int index() const { return current_idx; }

    // Moves the state machine to the next bin in the outward radial pattern
    void next() {
        step_y();
        advance();
    }

private:
    int target_row, target_col;
    int p_offset = 0, p_sign = 1;
    int y_offset = 0, y_sign = 1;
    int current_idx = -1;

    void advance() {
        // Expand pitch row by row until we've covered the whole vertical map
        while (p_offset <= Sensor::VFH_LATITUDE_BINS) {
            int row = target_row + (p_offset == 0 ? 0 : p_sign * p_offset);

            // Only sweep if the pitch row actually exists (avoiding the poles)
            if (row >= 0 && row < Sensor::VFH_LATITUDE_BINS) {
                
                // Sweep horizontally until we've done a full 360 (half left, half right)
                while (y_offset <= Sensor::VFH_AZIMUTH_BINS / 2) {
                    int col = target_col + (y_offset == 0 ? 0 : y_sign * y_offset);

                    // Pac-Man Azimuth Wrap
                    while (col < 0) col += Sensor::VFH_AZIMUTH_BINS;
                    col %= Sensor::VFH_AZIMUTH_BINS;

                    current_idx = (row * Sensor::VFH_AZIMUTH_BINS) + col;
                    return; // Pause the state machine and yield this index
                }
            }
            // If we swept the whole row, or the row was invalid, move to the next pitch row
            step_p();
        }
        // If we break the outer loop, we've searched the entire map.
        current_idx = -1; 
    }

    void step_y() {
        if (y_offset == 0) {
            y_offset = 1; y_sign = 1;
        } else if (y_sign == 1) {
            y_sign = -1; // Check the other direction
        } else {
            y_offset++; y_sign = 1; // Widen the horizontal search
        }
    }

    void step_p() {
        y_offset = 0; y_sign = 1; // Reset the horizontal sweep for the new row
        if (p_offset == 0) {
            p_offset = 1; p_sign = 1;
        } else if (p_sign == 1) {
            p_sign = -1;
        } else {
            p_offset++; p_sign = 1; // Widen the vertical search
        }
    }
};

void ReactiveOANode::computeCorrectionVector() {
    Eigen::Vector3f avoidance_vec = repulsive_vec + control_vec;
    Eigen::Vector3f target_vec = control_vec.squaredNorm() > avoidance_vec.squaredNorm() ? control_vec : avoidance_vec;
    float target_speed = target_vec.norm();
    if(target_speed < 1e-3f) {
        correction_vec.setZero();
        return;
    }

#if DEBUG
    Eigen::Vector3f spherical_target_vec = math_utils::toSpherical(target_vec);
    RCLCPP_INFO(
        this->get_logger(), GREEN "Target = %.2f, yaw = %.0f, pitch = %.0f" RESET, 
        spherical_target_vec.z(), 
        spherical_target_vec.x() / DEGREE, 
        spherical_target_vec.y() / DEGREE
    );
#endif

    Eigen::Vector3f target_direction = target_vec.normalized();
    Eigen::Vector3f movement_direction = movement_vec.squaredNorm() > 1e-6f ? movement_vec.normalized() : target_direction;

    // --- GOAL 3: INDEPENDENT FLU WEIGHTS ---
    // Penalize vector deviation based on physical drone mechanics
    constexpr float W_X = 1.0f; // Forward (Medium penalty for slowing down)
    constexpr float W_Y = 0.5f; // Left/Right (Low penalty, rolling is easy)
    constexpr float W_Z = 2.5f; // Up/Down (High penalty, fighting gravity is expensive)
    constexpr float W_MOMENTUM = 0.3f; // Maintain current trajectory

    // --- GOAL 4: HEURISTIC EARLY EXIT ---
    // If a bin scores lower than this, we stop searching instantly.
    // 0.0 is a perfect match. Tweak this higher to make the Spine "lazier" but faster.
    constexpr float ACCEPTABLE_COST = 0.2f; 

    // --- GOAL 1 & 2: OUTWARD HORIZONTAL-FIRST SEARCH ---
    constexpr int HORIZ_THRESH_BINS = 12; // 60 degrees left/right max swerve
    constexpr int VERT_THRESH_BINS = 6;   // 30 degrees up/down max swerve

    // Find Ground Zero (Target Bin)
    float t_yaw = std::atan2(target_vec.y(), target_vec.x());
    float t_pitch = std::atan2(-target_vec.z(), std::hypot(target_vec.x(), target_vec.y()));
    
    int target_col = static_cast<int>((t_yaw + M_PI) / Sensor::VFH_RESOLUTION);
    int target_row = static_cast<int>((t_pitch + M_PI_2) / Sensor::VFH_RESOLUTION);

    float min_cost = std::numeric_limits<float>::max();
    Eigen::Vector3f best_direction = Eigen::Vector3f::Zero();
    bool found_safe_path = false;

    // Find Ground Zero (Target Bin)
    float t_yaw = std::atan2(target_vec.y(), target_vec.x());
    float t_pitch = std::atan2(-target_vec.z(), std::hypot(target_vec.x(), target_vec.y()));
    
    int target_col = static_cast<int>((t_yaw + M_PI) / Sensor::VFH_RESOLUTION);
    int target_row = static_cast<int>((t_pitch + M_PI_2) / Sensor::VFH_RESOLUTION);

    float min_cost = std::numeric_limits<float>::max();
    Eigen::Vector3f best_direction = Eigen::Vector3f::Zero();
    bool found_safe_path = false;

    // --- THE NEW FLAT RADIAL SEARCH ---
    RadialVfhScanner scanner(target_row, target_col);

    while (scanner.index() != -1) {
        int i = scanner.index();

        // If blocked, advance the scanner and skip the math
        if (VFH[i]) {
            scanner.next();
            continue; 
        }

        // Decode the index back into row/col for math
        int row = i / Sensor::VFH_AZIMUTH_BINS;
        int col = i % Sensor::VFH_AZIMUTH_BINS;

        // Convert safe bin to Cartesian
        float yaw = ((col * Sensor::VFH_RESOLUTION) - M_PI + Sensor::VFH_RESOLUTION / 2.0f);
        float pitch = ((row * Sensor::VFH_RESOLUTION) - M_PI_2 + Sensor::VFH_RESOLUTION / 2.0f);
        Eigen::Vector3f candidate_direction = math_utils::toCartesian({yaw, pitch, 1.0f});

        // FLU Deviation Cost
        Eigen::Vector3f diff = candidate_direction - target_direction;
        float target_cost = (W_X * diff.x() * diff.x()) + 
                            (W_Y * diff.y() * diff.y()) + 
                            (W_Z * diff.z() * diff.z());

        float movement_cost = 1.0f - candidate_direction.dot(movement_direction);
        float total_cost = target_cost + (W_MOMENTUM * movement_cost);

        if (total_cost < min_cost) {
            min_cost = total_cost;
            best_direction = candidate_direction;
            found_safe_path = true;
            
            // EARLY EXIT: We found a path that is "good enough". 
            // Because we search outward radially, the first "good enough" path 
            // is guaranteed to be physically close to the target heading.
            if (min_cost <= ACCEPTABLE_COST) {
                break; 
            }
        }
        
        // Advance the scanner for the next iteration
        scanner.next(); 
    }
    
    // Panic Brake: No safe path found within our physical flight thresholds
    if(!found_safe_path) {
        correction_vec.setZero();
        return;
    }

    float deviation_factor = std::max(0.0f, best_direction.dot(target_direction));
    float safe_speed = target_speed * deviation_factor;
    correction_vec = best_direction * safe_speed;

#if DEBUG
    Eigen::Vector3f spherical_correction_vec = math_utils::toSpherical(correction_vec);
    RCLCPP_INFO(
        this->get_logger(), 
        YELLOW "Correction = %.2f, yaw = %.0f, pitch = %.0f" RESET, 
        spherical_correction_vec.z(), 
        spherical_correction_vec.x() / DEGREE, 
        spherical_correction_vec.y() / DEGREE
    );
#endif
}