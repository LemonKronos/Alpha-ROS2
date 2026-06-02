#!/bin/bash

GLOBAL_ENV="$HOME/MyCode/Project/Drone/ROS2/scripts/global.sh"

if [ -f "$GLOBAL_ENV" ]; then
    source "$GLOBAL_ENV"
else
    echo '[Missing Global ENV setup]'
fi

export FAST_DDS_SETUP="$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/fast_dds_setup.sh"
export SETUP_ROS2="$HOME/MyCode/Project/Drone/ROS2/scripts/setupROS2Terminal.sh"

# export GAZEBO_HEADLESS="--headless"

# ==========================================
# Functions
# ==========================================

start_gazebo() {
  (
    cd ~/MyCode/Project/Drone/Gazebo 
    exec rNvidia python3 simulation-gazebo --world "$WORLD_NAME" > /dev/null 2>&1 # rNvidia is a env set for Nvidia GPU on Linux
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

MODEL_POSE="0,0,1"
if [[ "$WORLD_NAME" == "grasslands" ]]; then
  MODEL_POSE="0,13,1"
fi
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
PX4_GZ_MODEL_POSE="$MODEL_POSE" \\
make px4_sitl "gz_$DRONE_NAME"
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

    ros2 launch launch_utils ros_gz_bridge_launch.py
  ' & echo $! >> /tmp/sim_pids.txt
  printf "\033[92m[GZ_ROS_BRIDGE] Running...\033[0m\n"
}

start_rviz2() {
  env DISPLAY="$DISPLAY" QT_QPA_PLATFORM=xcb zsh -c '
    source "$SETUP_ROS2"
    rNvidia rviz2
    exec zsh
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[Rviz2] running...\033[0m\n"
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
  start_rviz2

  sleep 5
  echo "Run $DRONE_NAME with world $WORLD_NAME" | cowsay -f duck | lolcat
}

# Run the script
main