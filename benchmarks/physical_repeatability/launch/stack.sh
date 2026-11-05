#!/usr/bin/env bash
# Start the minimal motion stack for the benchmark. Runs ONLY when invoked by the
# user (run.sh stack); never from tests, imports, help, recover or report.
#   /move_group running        -> nothing started
#   /controller_manager running -> move_group + add_ground only (no driver/RSP duplicate)
#   nothing running            -> full hardware.launch.py without RViz
# Never starts cameras, YOLO, depth/rectification or ball-pick nodes.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

nodes="$(ros2 node list --no-daemon --spin-time 3 2>/dev/null || true)"
if grep -qx '/move_group' <<<"$nodes"; then
  echo "move_group already running; nothing started."
  exit 0
fi
if grep -qx '/controller_manager' <<<"$nodes"; then
  echo "controller_manager present: starting move_group + add_ground only."
  echo "Arm controller must be active for motion: run.sh activate-arm (gripper controller untouched)."
  exec ros2 launch "$HERE/moveit_only.launch.py" "$@"
fi
if grep -qx '/robot_state_publisher' <<<"$nodes"; then
  echo "robot_state_publisher found without controller_manager: inconsistent stack, stop it first." >&2
  exit 1
fi
echo "No driver found: starting full hardware.launch.py (driver, controllers, RSP, MoveIt), RViz off."
echo "Note: its spawner activates gripper_controller, which holds the measured gripper position."
exec ros2 launch hiwonder_xarm_esp32_moveit_config hardware.launch.py use_rviz:=false "$@"
