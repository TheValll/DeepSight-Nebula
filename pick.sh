#!/usr/bin/env bash
# One command for repeated ball picks (ball_pick_hardware.launch.py must run).
cd "$(dirname "$0")" || exit 1
source "$HOME/ros2_robotics/install/setup.bash"
source app/install/setup.bash
source .venv/bin/activate
export DYLD_LIBRARY_PATH="$(brew --prefix libomp)/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-LARGE_DATA?max_msg_size=4MB&sockets_size=4MB}"
exec ros2 run stereo_camera_calibration pick "$@"
