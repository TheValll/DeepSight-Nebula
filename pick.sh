#!/usr/bin/env bash
# One command for repeated ball picks (ball_pick_hardware.launch.py must run).
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
source app/install/setup.bash
exec ros2 run stereo_camera_calibration pick "$@"
