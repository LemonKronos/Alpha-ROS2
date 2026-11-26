#!/bin/bash
# Restart ONLY the Gazebo server (headless). The ros_gz_bridge and the pipeline keep running.
# Exit code 0 when the new server publishes /clock, 1 otherwise.

SCRIPT_FOLDER="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_FOLDER/startSimulation.sh"   # defines start_gazebo, main is not run when sourced

SERVER_PATTERN='gz sim -r .* -s$'
STOP_TIMEOUT_SECONDS=10
OLD_CLOCK_TIMEOUT_SECONDS=15
NEW_CLOCK_TIMEOUT_SECONDS=60

#: Do not reset if stop have been called
if ! pgrep -x MicroXRCEAgent > /dev/null; then
  exit 1

#: Stop the old server: SIGINT first, SIGKILL if it hangs
echo "[RESTART GZ] Stopping the Gazebo server..."
pkill -INT -f "$SERVER_PATTERN"
for ((second = 0; second < STOP_TIMEOUT_SECONDS; second++)); do
  pgrep -f "$SERVER_PATTERN" >/dev/null || break
  sleep 1
done
if pgrep -f "$SERVER_PATTERN" >/dev/null; then
  echo "[RESTART GZ] Server did not stop, killing it"
  pkill -KILL -f "$SERVER_PATTERN"
  sleep 1
fi

#: Wait until the old /clock is gone, so we do not mistake it for the new server
for ((second = 0; second < OLD_CLOCK_TIMEOUT_SECONDS; second++)); do
  gz topic -l 2>/dev/null | grep -q "/clock" || break
  sleep 1
done

#: Start the new server and wait for its /clock
start_gazebo
for ((second = 0; second < NEW_CLOCK_TIMEOUT_SECONDS; second++)); do
  if gz topic -l 2>/dev/null | grep -q "/clock"; then
    pgrep -f "$SERVER_PATTERN" >> /tmp/sim_pids.txt
    echo "[RESTART GZ] Gazebo is back"
    exit 0
  fi
  sleep 1
done

echo "[RESTART GZ] Gazebo did not publish /clock in time"
exit 1