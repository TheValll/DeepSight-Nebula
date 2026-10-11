#!/usr/bin/env python3
"""Recreate the temporary stereo reference used for the 2026-10-01 ball picks.

Uses the same saved checkerboard images and OpenCV settings as the recorded
2026-09-30 calibration command. Does not access the camera or move the robot.
The original stereo_calib.xml is preserved; existing output is never overwritten.
"""
from pathlib import Path

import cv2
import numpy as np


def main():
    calibration = Path(__file__).resolve().parents[2] / "calibration"
    output = calibration / "stereo_calib_standard.xml"
    if output.exists():
        raise FileExistsError(output)
    left_paths = sorted((calibration / "images/left").glob("*.png"))
    right_paths = sorted((calibration / "images/right").glob("*.png"))
    if not left_paths or [p.name for p in left_paths] != [p.name for p in right_paths]:
        raise ValueError("Expected matching saved left/right checkerboard images")
    object_grid = np.zeros((40, 3), np.float32)
    object_grid[:, :2] = np.mgrid[:8, :5].T.reshape(-1, 2)*.025
    objects, left_points, right_points = [], [], []
    for left_path, right_path in zip(left_paths, right_paths):
        left = cv2.imread(str(left_path), 0)
        right = cv2.imread(str(right_path), 0)
        if left is None or right is None or left.shape != (720, 1280) or right.shape != (720, 1280):
            raise ValueError(f"Invalid checkerboard image pair: {left_path}")
        found_left, corners_left = cv2.findChessboardCorners(left, (8, 5), 3)
        found_right, corners_right = cv2.findChessboardCorners(right, (8, 5), 3)
        if found_left and found_right:
            objects.append(object_grid.copy())
            left_points.append(cv2.cornerSubPix(left, corners_left, (11, 11), (-1, -1), (3, 30, .01)))
            right_points.append(cv2.cornerSubPix(right, corners_right, (11, 11), (-1, -1), (3, 30, .01)))
    if len(objects) < 3:
        raise ValueError("Too few usable checkerboard pairs")
    rms, k1, d1, k2, d2, rotation, translation, _, _ = cv2.stereoCalibrate(
        objects, left_points, right_points, None, None, None, None, (1280, 720),
        criteria=(3, 100, 1e-5), flags=0,
    )
    if not np.isfinite(rms) or not all(np.isfinite(a).all() for a in (k1, d1, k2, d2, rotation, translation)):
        raise ValueError("Calibration contains non-finite results")
    storage = cv2.FileStorage(str(output), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise OSError(f"Could not create calibration: {output}")
    try:
        for key, value in (("K1", k1), ("D1", d1), ("K2", k2), ("D2", d2), ("R", rotation), ("T", translation)):
            storage.write(key, value)
        storage.write("RMS", rms)
    finally:
        storage.release()
    print(f"Restored {output}: {len(objects)} pairs, RMS={rms:.6f}, baseline={np.linalg.norm(translation):.6f} m")


if __name__ == "__main__":
    main()
