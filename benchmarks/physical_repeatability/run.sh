#!/usr/bin/env bash
# Physical repeatability benchmark launcher. Nothing is started unless asked.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
PYTHON="${PYTHON:-$ROOT/.venv/bin/python}"

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source "$HOME/ros2_robotics/install/setup.bash"
  # shellcheck disable=SC1091
  source "$ROOT/app/install/setup.bash"
  set -u
}

usage() {
  cat <<EOF
Usage: run.sh [run] [options]        execute cycles (MOVES THE ROBOT)
       run.sh preflight [--live]     checks only; --live reads joints/cameras and plans, never executes
       run.sh stack                  start minimal MoveIt (or full hardware.launch if no driver)
       run.sh activate-arm           activate arm_controller only (gripper controller untouched)
       run.sh status                 list ROS nodes and controllers
       run.sh recover SESSION        offline recovery/export (no ROS, no motion)
       run.sh report SESSION         offline statistics report (no ROS)
       run.sh test                   offline unit/mock/crash tests (no hardware)

Defaults: 50 cycles, velocity 1.0, acceleration 0.10, both cameras, TCP + marker-8 values.
EOF
}

command="${1:-run}"
case "$command" in
  -h|--help|help)
    usage; echo; exec "$PYTHON" "$HERE/benchmark.py" --help ;;
  recover)  shift; exec "$PYTHON" "$HERE/recover.py" "$@" ;;
  report)   shift; exec "$PYTHON" "$HERE/report.py" "$@" ;;
  test)     shift; cd "$HERE"; exec "$PYTHON" -m unittest discover -s tests -t . -v "$@" ;;
  stack)    shift; source_ros; exec "$HERE/launch/stack.sh" "$@" ;;
  activate-arm)
    # Jazzy JTC duplicates its command interfaces when re-configured in place
    # ("Expected 5 'position' command interfaces, got 10"): unload + reload instead.
    # Activation holds the current position; the gripper controller is untouched.
    source_ros
    if ros2 control list_controllers | grep -E '^arm_controller\b' | grep -qw active; then
      echo "arm_controller already active."; exit 0
    fi
    ros2 control unload_controller arm_controller || true
    ros2 control load_controller --set-state active arm_controller
    sleep 1
    ros2 control list_controllers | grep -E '^arm_controller\b' | grep -qw active \
      || { echo "arm_controller not active: check joint limits in controller_manager log (~/.ros/log/ros2_control_node_*.log)" >&2; exit 1; }
    ros2 control list_controllers ;;
  status)   source_ros; ros2 node list; ros2 control list_controllers ;;
  run|preflight) source_ros; exec "$PYTHON" "$HERE/benchmark.py" "$@" ;;
  -*)       source_ros; exec "$PYTHON" "$HERE/benchmark.py" run "$@" ;;
  *)        usage >&2; exit 2 ;;
esac
