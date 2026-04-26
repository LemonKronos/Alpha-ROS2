#!/bin/bash

#_ ROS2
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"

#_ NAME
export WORLD_NAME="grasslands"
# export WORLD_NAME="obstacle_tunnel"

# export DRONE_NAME="alpha_minus_1"
export DRONE_NAME="alpha_minus_2"
# export DRONE_NAME="standard_vtol"
# export DRONE_NAME="tiltrotor"
# export DRONE_NAME="x500"

export RECORDED_MANUEVER_NAME="obstacle_tunnel_demo"

#_ PATH
export RECORD_STORAGE_PATH="$HOME/MyCode/Project/Drone/AIBrain/datasets/acrobatic_oa_dataset/obstacle_tunnel"
