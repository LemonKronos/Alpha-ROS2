#!/bin/bash

echo "[STOP SIM] Stopping safely..."


#_ 1. Kill PIDs stored in /tmp/sim_pids.txt
if [[ -f /tmp/sim_pids.txt ]]; then
  while read -r PID; do
    # Added safety: don't kill the script itself or its parent shell
    if [[ -n "$PID" ]] && [[ "$PID" != "$$" ]] && [[ "$PID" != "$PPID" ]]; then
      if kill -0 "$PID" 2>/dev/null; then
        kill "$PID" 2>/dev/null
        echo "Killed PID from list: $PID"
      fi
    fi
  done < /tmp/sim_pids.txt
  rm /tmp/sim_pids.txt
fi


#_ 2. Explicit PX4 processes
pkill -x px4 >/dev/null                           # exact binary name
pkill -x micrortps_agent >/dev/null               # px4 agent name
pkill -x MicroXRCEAgent >/dev/null                # alternative agent


#_ Gazebo processes
pkill -INT -f 'gz sim -r .* -s$'  # Server
pkill -KILL -f '^gz sim -g$'      # Client


#_ ros_gz_bridge parameter bridge
pkill -f "parameter_bridge" >/dev/null 


#_ 4. ROS visualization tools
pkill -x rviz2 >/dev/null


#_ 5. QGroundControl (GUI app)
pgrep -x QGroundControl >/dev/null && pkill -9 -x QGroundControl >/dev/null


#_ 6. Close web control emulator
hyprctl clients -j | jq -r '.[] | select(.class == "brave-browser" and .title == "Alpha HTML Controller - Brave") | .address' | xargs -r -I {} hyprctl dispatch closewindow address:{}


#_ 7. ROS 2 daemon and stale Fast DDS shared memory (never touches files a live node still uses)
pkill -f _ros2_daemon >/dev/null 2>&1
sleep 1
for shared_memory_file in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
  [[ -e "$shared_memory_file" ]] && ! fuser -s "$shared_memory_file" 2>/dev/null && rm -f "$shared_memory_file"
done


echo "[STOP SIM] Cleanup completed safely!"

