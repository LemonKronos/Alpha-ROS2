#!/bin/bash

#_ SET UP 
GLOBAL_ENV="$HOME/MyCode/Project/Drone/ROS2/scripts/global.sh"

if [ -f "$GLOBAL_ENV" ]; then
    source "$GLOBAL_ENV"
else
    echo '[Missing Global ENV setup]'
fi

export FAST_DDS_SETUP="$HOME/MyCode/Project/Drone/ROS2/config/FastDDS/fast_dds_setup.sh"
export SETUP_ROS2="$HOME/MyCode/Project/Drone/ROS2/scripts/setupROS2Terminal.sh"

#_ GAZEBO SERVER
start_gazebo() {
  (
    cd "$HOME/MyCode/Project/Drone/Gazebo" || exit 1
    exec rNvidia python3 simulation-gazebo \
      --world "$WORLD_NAME" --headless
  ) >/dev/null 2>&1 &

  echo $! >> /tmp/sim_pids.txt
  printf "\033[92m[GZ SERVER] Running...\033[0m\n"
}

#_ GAZEBO GUI
start_gazebo_gui() {
  (
    cd "$HOME/MyCode/Project/Drone/Gazebo" || exit 1

    # Wait until the server publishes its simulation clock.
    until gz topic -l 2>/dev/null | grep -q "/clock"; do
      sleep 1
    done

    exec rNvidia python3 simulation-gazebo --gui
  ) >/dev/null 2>&1 &

  # Intentionally do NOT store the GUI PID.
  printf "\033[92m[GZ GUI] Launching...\033[0m\n"
}

#_ MICRO XRCE AGENT
start_micro_xrce_agent() {
  zsh -c '
    source "$FAST_DDS_SETUP"
    
    cd ~/MyCode/Project/Drone/Micro-XRCE-DDS-Agent
    export PATH=$PATH:$HOME/MyCode/Project/Drone/Micro-XRCE-DDS-Agent/build
    
    MicroXRCEAgent udp4 -p 8888 > /tmp/micro_xrce_agent.log 2>&1
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[MicroXRCEAgent] Running ...\033[0m\n"
}

#_ PX4
MODEL_POSE="0,0,1"
if [[ "$WORLD_NAME" == "grasslands" ]]; then
  MODEL_POSE="0,13,0.5"
fi
start_px4_sitl() {
  kitty --title "PX4 Autopilot" env TERMINAL_TAG=PX4 zsh -c "$(cat <<EOF
cd ~/MyCode/Project/Drone/PX4-Autopilot &&

mkdir -p build/px4_sitl_default/rootfs/fs/microsd/etc &&
echo "param set UXRCE_DDS_SYNCT 0" > build/px4_sitl_default/rootfs/fs/microsd/etc/extras.txt &&

MAV_0_CONFIG=0 \\
PX4_GZ_STANDALONE=1 \\
PX4_GZ_WORLD="$WORLD_NAME" \\
# PX4_SIM_SPEED_FACTOR=0.5 \\
PX4_HOME_LAT=10.8776 \\
PX4_HOME_LON=106.8071 \\
PX4_HOME_ALT=0 \\
PX4_GZ_MODEL_POSE="$MODEL_POSE" \\
make px4_sitl "gz_$DRONE_NAME"
exec zsh
EOF
)" & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[PX4] Starting with Gazebo time sync in standalone mode...\033[0m\n"
}

#_ QGROUND CONTROL
start_qgroundcontrol() {
  zsh -c '
    QGroundControl-x86_64.AppImage > /dev/null 2>&1
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[QGroundControl] running...\033[0m\n"
}

#_ Web Socket Control
start_web_control() {
  brave-browser /home/mr_lemon/MyCode/Project/Drone/ROS2/src/Control/advance_control/resource/controller.html >/dev/null 2>&1 &
  printf "\033[92m[Web Socket Control] opening...\033[0m\n"
}

#_ ROS2 TERMINAL
start_ros2_terminal() {
  kitty zsh -c '
    source "$SETUP_ROS2"
    exec zsh
  ' & echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[ROS2 Terminal] running...\033[0m\n"
}

#_ ROS GZ BRIDGE
start_gz_ros_bridge() {
  zsh -c '
    source "$SETUP_ROS2"
    
    # Wait for clock to ensure GZ is up
    until gz topic -l | grep -q "/clock"; do sleep 1; done

    ros2 launch launch_utils ros_gz_bridge_launch.py
  ' >/dev/null 2>&1 &
  echo $! >> /tmp/sim_pids.txt
  printf "\033[92m[GZ_ROS_BRIDGE] Running...\033[0m\n"
}

#_ RVIZ2
start_rviz2() {
  env DISPLAY="$DISPLAY" QT_QPA_PLATFORM=xcb zsh -c '
    source "$SETUP_ROS2"
    rNvidia rviz2
    exec zsh
  ' >/dev/null 2>&1 &
  echo $! >> /tmp/sim_pids.txt &&
  printf "\033[92m[Rviz2] running...\033[0m\n"
}

#_ MAIN FUNCTION
main() {
  # Clear previous PIDs just in case
  > /tmp/sim_pids.txt

  source "$FAST_DDS_SETUP"

  start_gazebo
  start_micro_xrce_agent
  start_px4_sitl
  # start_qgroundcontrol
  start_ros2_terminal
  start_gz_ros_bridge
  sleep 1
  start_web_control
  sleep 1
  start_gazebo_gui
  start_rviz2

  sleep 1
  echo "Run $DRONE_NAME with world $WORLD_NAME" | cowsay -f duck | lolcat
}

#_ Run the script
main