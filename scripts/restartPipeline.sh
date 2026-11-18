#!/bin/bash
# Restart the pipeline launch file. Gazebo, the bridge and the trainer keep running.
# Exit code 0 when every required node is back, 1 otherwise.

GLOBAL_ENV="$HOME/MyCode/Project/Drone/ROS2/scripts/global.sh"
[ -f "$GLOBAL_ENV" ] && source "$GLOBAL_ENV"
export SETUP_ROS2="$HOME/MyCode/Project/Drone/ROS2/scripts/setupROS2Terminal.sh"
export KILL_LIST="/tmp/sim_pids.txt"

LAUNCH_PATTERN="ros2 launch launch_utils acrobatic_training.py"
LAUNCH_LOG="/tmp/acrobatic_training_launch.log"
REQUIRED_NODES=(finalize_control fuse_perception contact_parser gz_ground_truth px4_emulator)
PIPELINE_PROCESS_PATTERN="static_transform_publisher|component_container_mt|lib/(finalize_control|fuse_perception|contact_parser|gz_ground_truth|px4_emulator)/"
STOP_TIMEOUT_SECONDS=15
SURVIVOR_TIMEOUT_SECONDS=5
GONE_TIMEOUT_SECONDS=15
LAUNCH_SETTLE_SECONDS=2

visible_required_node_count() {
  local running_nodes
  local count=0
  running_nodes=$(zsh -c 'source "$SETUP_ROS2"; ros2 node list --no-daemon' 2>/dev/null)
  for required_node in "${REQUIRED_NODES[@]}"; do
    grep -qx "/$required_node" <<< "$running_nodes" && count=$((count + 1))
  done
  echo "$count"
}

# Prints every PID the launch reported, from lines like: "process started with pid [302925]"
read_launch_log_process_ids() {
  [[ -f "$LAUNCH_LOG" ]] && grep -oP 'process started with pid \[\K[0-9]+' "$LAUNCH_LOG"
}

# A PID from an old log may have been reused by an unrelated process: only touch pipeline processes
is_pipeline_process() {
  ps -o args= -p "$1" 2>/dev/null | grep -qE "$PIPELINE_PROCESS_PATTERN"
}

# SIGINT first, then SIGKILL for whatever is still alive after SURVIVOR_TIMEOUT_SECONDS
stop_processes() {
  local process_ids="$1"
  local process_id
  local any_alive

  [[ -z "${process_ids// /}" ]] && return 0

  for process_id in $process_ids; do
    kill -INT "$process_id" 2>/dev/null
  done

  for ((second = 0; second < SURVIVOR_TIMEOUT_SECONDS; second++)); do
    any_alive=false
    for process_id in $process_ids; do
      kill -0 "$process_id" 2>/dev/null && any_alive=true
    done
    $any_alive || return 0
    sleep 1
  done

  for process_id in $process_ids; do
    kill -0 "$process_id" 2>/dev/null && kill -KILL "$process_id"
  done
}

#_ Read the old launch's process list BEFORE the log is overwritten
echo "[RESTART PIPELINE] Stopping the launch file..."
old_process_ids=$(read_launch_log_process_ids)

#_ SIGINT the launch (like Ctrl-C) so it shuts its nodes down cleanly
old_launch_process_ids=$(pgrep -f "$LAUNCH_PATTERN")
for launch_process_id in $old_launch_process_ids; do
  kill -INT "$launch_process_id"
done

for ((second = 0; second < STOP_TIMEOUT_SECONDS; second++)); do
  pgrep -f "$LAUNCH_PATTERN" >/dev/null || break
  sleep 1
done

if pgrep -f "$LAUNCH_PATTERN" >/dev/null; then
  echo "[RESTART PIPELINE] Launch did not stop in ${STOP_TIMEOUT_SECONDS}s, killing it"
  kill -KILL $(pgrep -f "$LAUNCH_PATTERN")
fi

#_ Clean up whatever survived of the old nodes: first the PIDs from the log, then a name sweep as a safety net
survivor_process_ids=""
for process_id in $old_process_ids; do
  is_pipeline_process "$process_id" && survivor_process_ids="$survivor_process_ids $process_id"
done
stop_processes "$survivor_process_ids"
pkill -KILL -f "$PIPELINE_PROCESS_PATTERN"

#_ Wait until the old nodes are gone from the ROS graph
for ((second = 0; second < GONE_TIMEOUT_SECONDS; second++)); do
  [[ "$(visible_required_node_count)" == "0" ]] && break
  sleep 1
done

#_ Start the launch file again, detached from the trainer. $$ is the PID that exec keeps for ros2 launch.
echo "[RESTART PIPELINE] Starting the launch file..."
setsid zsh -c 'echo $$ >> "$KILL_LIST"; source "$SETUP_ROS2"; exec ros2 launch launch_utils acrobatic_training.py' \
  > "$LAUNCH_LOG" 2>&1 < /dev/null &

#_ Give the launch time to start its processes, then put every PID it reported on the kill list
sleep "$LAUNCH_SETTLE_SECONDS"
new_process_ids=$(read_launch_log_process_ids)

if [[ -z "$new_process_ids" ]]; then
  echo "[RESTART PIPELINE] No 'process started with pid' lines found in $LAUNCH_LOG"
  exit 1
fi

echo "$new_process_ids" >> "$KILL_LIST"
echo "[RESTART PIPELINE] Launch started, recorded: $(echo $new_process_ids)"
exit 0