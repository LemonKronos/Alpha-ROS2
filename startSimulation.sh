#!/bin/bash

WORLD_NAME="grasslands"

DRONE_NAME="gz_alpha_minus_1"
# DRONE_NAME="gz_standard_vtol"
# DRONE_NAME="gz_tiltrotor"

# === No1: Custom Gazebo world ===
cd ~/MyCode/Project/Drone/Gazebo 
python3 simulation-gazebo --world "$WORLD_NAME" > /dev/null 2>&1 & disown &&
printf "\033[92m[GZ SIM] running...\033[0m\n"

# === No2: Micro XRCE-DDS Agent ===
(
  cd ~/MyCode/Project/Drone/Micro-XRCE-DDS-Agent
  source /opt/ros/jazzy/setup.bash
  export PATH=$PATH:$HOME/MyCode/Project/Drone/Micro-XRCE-DDS-Agent/build
  MicroXRCEAgent udp4 -p 8888 -l 127.0.0.1 > /dev/null 2>&1
) & echo $! >> /tmp/sim_pids.txt &&
printf "\033[92m[MicroXRCEAgent] Running silently on UDP port 8888...\033[0m\n"


# === No3: PX4 SITL with Gazebo RTPS , stand alone mode ===
kitty --title "PX4 Autopilot" env TERMINAL_TAG=PX4 zsh -c "$(cat <<EOF
cd ~/MyCode/Project/Drone/PX4-Autopilot &&
MAV_0_CONFIG=0 \\
PX4_GZ_STANDALONE=1 \\
PX4_GZ_WORLD="$WORLD_NAME" \\
# PX4_SIM_SPEED_FACTOR=0.5 \\
PX4_HOME_LAT=10.8776 \\
PX4_HOME_LON=106.8071 \\
PX4_HOME_ALT=101 \\
PX4_GZ_MODEL_POSE="0,0,8.5" \\
make px4_sitl "$DRONE_NAME"
exec zsh
EOF
)" & echo $! >> /tmp/sim_pids.txt &&
printf "\033[92m[PX4] Starting with Gazebo in standalone mode...\033[0m\n"

# === No4: Run QGroundControl ===
QGroundControl-x86_64.AppImage > /dev/null 2>&1 & disown &&
printf "\033[92m[QGroundControl] running...\033[0m\n"

# === No5: Run a basic ROS 2 terminal ===
kitty env TERMINAL_TAG=ROS2 zsh -c '
printf "[ROS 2] Launching sourced terminal..." ;
source /opt/ros/jazzy/setup.zsh ;
cd ~/MyCode/Project/Drone/ROS2 &&
source install/setup.zsh ;
source /home/mr_lemon/MyCode/Project/Drone/ROS2/pyvenv/bin/activate;
exec zsh' & echo $! >> /tmp/sim_pids.txt &&
printf "\033[92m[ROS2 Terminal] running...\033[0m\n"

# === No6: Run Gazebo ROS2 bridge ===
(
  source /opt/ros/jazzy/setup.bash
  cd ~/MyCode/Project/Drone/ROS2
  source install/setup.bash
  until gz topic -l | grep -q "/clock"; do sleep 1; done
  CONFIG_FILE="$HOME/MyCode/Project/Drone/ROS2/config/gz_ros_bridge/${WORLD_NAME}-${DRONE_NAME}.YAML"
  if [[ -f "$CONFIG_FILE" ]]; then
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$CONFIG_FILE"
  else
    printf "\033[33m[GZ_ROS2_BRIDGE] No config file for this run!\033[0m\n"
    echo "$WORLD_NAME-$DRONE_NAME.YAML - file = $CONFIG_FILE"
    exit 1
  fi
) & echo $! >> /tmp/sim_pids.txt &&
printf "\033[92m[GZ_ROS_BRIDGE] Running...\033[0m\n"


# === No7: Run Rviz2 ===
# kitty --detach --title "PX4 Autopilot" env TERMINAL_TAG=PX4 zsh -c '
# source /opt/ros/jazzy/setup.zsh &&
# cd ~/MyCode/Project/Drone/ROS2 &&
# source install/setup.zsh ;
# rNvidia rviz2' & echo $! >> /tmp/sim_pids.txt &&
# printf "\033[92m[ROS2 Rviz2] running...\033[0m\n"


sleep 5
echo "Run $DRONE_NAME with world $WORLD_NAME" | lolcat