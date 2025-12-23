import numpy as np
import torch
import python_utils.utils as sys_config

class RoboticNormalizer:
    def __init__(self, device):
        self.device = device
        
        # Metadata from Global System Config
        self.max_v = sys_config.SPEED_MAX_FORWARD
        self.max_w = 3.14 # Approximate max angular velocity (often not in config, but kept safe)
        self.max_lidar_down = sys_config.LIDAR_2D_RANGE_MAX # Assuming similar range
        self.max_lidar_2d = sys_config.LIDAR_2D_RANGE_MAX
        
        # Delta calc helpers
        self.dt = sys_config.SYSTEM_CYCLE
        self.max_dist_frame = (self.max_v * self.dt) * 1.5
        if self.max_dist_frame < 1e-6: self.max_dist_frame = 1.0

        # Stateful history for Delta Position
        self.prev_pos = None

    def normalize_noob(self, action_arr):
        """
        Action/Noob (7 dim)
        Clamps 0-5 to [-1, 1]. Leaves 6 (Wings) alone.
        Expected Input: Numpy Array [7]
        """
        norm = action_arr.copy()
        
        # 0-5: Controls (Clamp)
        norm[0:6] = np.clip(norm[0:6], -1.0, 1.0)
        
        # Convert to tensor [1, 7]
        return torch.from_numpy(norm).float().unsqueeze(0).to(self.device)

    def normalize_state(self, state_arr):
        """
        State (40 dim) - ROBOTIC SPECIFIC
        Input: Numpy Array [40]
        Output: Tensor [1, 40]
        """
        norm = state_arr.copy()

        # --- 1. Delta Position (0-2) ---
        current_pos = norm[0:3]
        
        if self.prev_pos is None:
            # First frame: No movement
            delta_pos = np.zeros(3)
        else:
            delta_pos = current_pos - self.prev_pos
            
        # Update history
        self.prev_pos = current_pos
        
        # Normalize Delta
        norm[0:3] = np.clip(delta_pos / self.max_dist_frame, -1.0, 1.0)

        # --- 2. Quat (3-6) ---
        # Keep as is

        # --- 3. Velocity (7-9) ---
        norm[7:10] = np.clip(norm[7:10] / self.max_v, -1.0, 1.0)

        # --- 4. Ang Vel (10-12) ---
        norm[10:13] = np.clip(norm[10:13] / self.max_w, -1.0, 1.0)

        # --- 5. Contacts (13-14) ---
        # Keep as is

        # --- 6. Lidar Down (15) ---
        norm[15] = np.clip(norm[15] / self.max_lidar_down, 0.0, 1.0)

        # --- 7. Lidar 2D (16-39) ---
        norm[16:40] = np.clip(norm[16:40] / self.max_lidar_2d, 0.0, 1.0)

        # To Tensor [1, 40]
        return torch.from_numpy(norm).float().unsqueeze(0).to(self.device)

    def denormalize_action(self, action_tensor):
        """
        Safeguard Logic.
        Clamps the model output to ensure we don't send garbage.
        """
        out = action_tensor.clone()
        
        # 0-5: Rates, Velocities -> Clamp to [-1, 1]
        out[:, :, 0:6] = torch.clamp(out[:, :, 0:6], -1.0, 1.0)
        
        return out