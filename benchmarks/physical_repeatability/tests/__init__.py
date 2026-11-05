"""Offline tests. Importing this package forbids ROS and camera access in-process."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# Any attempt to import ROS or the robot interface fails loudly.
for _name in ('rclpy', 'robot', 'moveit_msgs', 'cv_bridge'):
    sys.modules[_name] = None

import cv2  # noqa: E402


class _NoCamera:
    def __init__(self, *args, **kwargs):
        raise AssertionError(f'tests must not open cameras: {args}')


cv2.VideoCapture = _NoCamera
