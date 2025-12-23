import numpy as np
from ros2_msgs.msg import ControlInterface

# Specific Utils for the Acrobatic Brain Node
# Logic for safety checks and emergency messages

def has_nan(data):
    """
    Checks if a numpy array, tensor, or list contains NaNs.
    """
    if data is None:
        return True
    return np.isnan(np.sum(data))

def get_safety_hover_msg(clock_now):
    """
    Returns a ControlInterface message that forces a Hover.
    Used when inputs are corrupted (NaN).
    """
    msg = ControlInterface()
    msg.header.stamp = clock_now.to_msg()
    msg.header.frame_id = "safety_hover"
    
    msg.control_by = ControlInterface.ACROBATIC_OA
    msg.control_state = True
    
    # HOVER COMMAND (0 Velocity, 0 Rates)
    msg.roll = 0.0
    msg.pitch = 0.0
    msg.yaw = 0.0
    msg.forward = 0.0
    msg.left = 0.0
    msg.up = 0.0
    
    # Keep wings mode as Multicopter for safety
    msg.wings_mode = ControlInterface.MULTICOPTER
    
    return msg