#!/bin/bash

# ==========================================
# Global Variables
# ==========================================
export WORLD_NAME="grasslands"
# export WORLD_NAME="obstacle_tunnel"

# export DRONE_NAME="gz_alpha_minus_1"
export DRONE_NAME="gz_alpha_minus_2"
# export DRONE_NAME="gz_standard_vtol"
# export DRONE_NAME="gz_tiltrotor"
# export DRONE_NAME="gz_x500"

export FAST_DDS_SETUP="$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/fast_dds_setup.sh"
export SETUP_ROS2="$HOME/MyCode/Project/Drone/ROS2/scripts/setupROS2Terminal.sh"

# export GAZEBO_HEADLESS="--headless"

# ==========================================
# Functions
# ==========================================

start_gazebo() {
  (
    cd ~/MyCode/Project/Drone/Gazebo 
    exec rNvidia python3 simulation-gazebo --world "$WORLD_NAME" "$GAZEBO_HEADLESS" > /dev/null 2>&1 # rNvidia is a env set for Nvidia GPU on Linux
  ) & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[GZ SIM] running...\033[0m\n"
}

start_micro_xrce_agent() {
  zsh -c '
    source "$FAST_DDS_SETUP"
    
    cd ~/MyCode/Project/Drone/Micro-XRCE-DDS-Agent
    export PATH=$PATH:$HOME/MyCode/Project/Drone/Micro-XRCE-DDS-Agent/build
    
    MicroXRCEAgent udp4 -p 8888 > /tmp/micro_xrce_agent.log 2>&1
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[MicroXRCEAgent] Running ...\033[0m\n"
}

start_px4_sitl() {
  kitty --title "PX4 Autopilot" env TERMINAL_TAG=PX4 zsh -c "$(cat <<EOF
cd ~/MyCode/Project/Drone/PX4-Autopilot &&
MAV_0_CONFIG=0 \\
PX4_GZ_STANDALONE=1 \\
PX4_GZ_WORLD="$WORLD_NAME" \\
# PX4_SIM_SPEED_FACTOR=0.25 \\
PX4_HOME_LAT=10.8776 \\
PX4_HOME_LON=106.8071 \\
PX4_HOME_ALT=0 \\
PX4_GZ_MODEL_POSE="0,13,1" \\
make px4_sitl "$DRONE_NAME"
exec zsh
EOF
)" & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[PX4] Starting with Gazebo in standalone mode...\033[0m\n"
}

start_qgroundcontrol() {
  zsh -c '
    QGroundControl-x86_64.AppImage > /dev/null 2>&1
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[QGroundControl] running...\033[0m\n"
}

start_ros2_terminal() {
  kitty zsh -c '
    source "$SETUP_ROS2"
    exec zsh
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[ROS2 Terminal] running...\033[0m\n"
}

start_gz_ros_bridge() {
  zsh -c '
    source "$SETUP_ROS2"
    
    # Wait for clock to ensure GZ is up
    until gz topic -l | grep -q "/clock"; do sleep 1; done
    
    CONFIG_FILE="$HOME/MyCode/Project/Drone/ROS2/config/gz_ros_bridge/${WORLD_NAME}-${DRONE_NAME}.YAML"

    if [[ -f "$CONFIG_FILE" ]]; then
      ros2 run ros_gz_bridge parameter_bridge \
        "/world/${WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld" \
        "/world/${WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity" \
        "/world/${WORLD_NAME}/remove@ros_gz_interfaces/srv/DeleteEntity" \
        "/world/${WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
        --ros-args -p config_file:="$CONFIG_FILE"
    else
      printf "\033[33m[GZ_ROS2_BRIDGE] No config file for this run!\033[0m\n"
      echo "$WORLD_NAME-$DRONE_NAME.YAML - file = $CONFIG_FILE"
      exit 1
    fi
  ' & echo $! >> /tmp/sim_pids.txt
  printf "\033[92m[GZ_ROS_BRIDGE] Running...\033[0m\n"
}

# ==========================================
# Main Execution
# ==========================================
main() {
  # Clear previous PIDs just in case
  > /tmp/sim_pids.txt

  source "$FAST_DDS_SETUP"

  start_gazebo
  start_micro_xrce_agent
  start_px4_sitl
  start_qgroundcontrol
  start_ros2_terminal
  start_gz_ros_bridge

  sleep 5
  echo "Run $DRONE_NAME with world $WORLD_NAME" | cowsay -f duck | lolcat
}

# Run the script
main