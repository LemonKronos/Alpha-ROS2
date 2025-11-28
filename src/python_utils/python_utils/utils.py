import rclpy
from rclpy.parameter import Parameter

def setup_for_simulation(node):
    # Check if parameter exists
    if node.has_parameter("use_sim_time"):
        # Force set to True
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    else:
        # Declare it as True
        node.declare_parameter("use_sim_time", True)