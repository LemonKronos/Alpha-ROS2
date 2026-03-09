#!/bin/bash

echo "[STOP SIM] Stopping safely..."

### 1. Kill PIDs stored in /tmp/sim_pids.txt
if [[ -f /tmp/sim_pids.txt ]]; then
  while read -r PID; do
    if kill -0 "$PID" 2>/dev/null; then
      kill "$PID" 2>/dev/null
      echo "Killed PID from list: $PID"
    fi
  done < /tmp/sim_pids.txt

  rm /tmp/sim_pids.txt
fi


### 2. Explicit PX4 processes
pkill -x px4 >/dev/null                           # exact binary name
pkill -x micrortps_agent >/dev/null               # px4 agent name
pkill -x MicroXRCEAgent >/dev/null                # alternative agent

### Gazebo classic processes
pkill -9 -f gz >/dev/null
pkill -f gazebo >/dev/null
pkill -f simulation-gazebo >/dev/null

### ros_gz_bridge parameter bridge
pkill -f "ros_gz_bridge parameter_bridge" >/dev/null 
pkill -f parameter_bridge >/dev/null


### 4. ROS visualization tools
pkill -x rviz2 >/dev/null

### 5. QGroundControl (GUI app)
pgrep -x QGroundControl >/dev/null && pkill -9 -x QGroundControl >/dev/null


echo "[STOP SIM] Cleanup completed safely!"
