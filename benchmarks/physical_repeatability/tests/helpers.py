import copy
import json
from pathlib import Path
import tempfile

import numpy as np

import settings

LOGITECH_K = np.array([[1426.829, 0, 994.096], [0, 1426.355, 505.216], [0, 0, 1]], float)


def rot(axis, degrees):
    import cv2
    vector = np.asarray(axis, float)
    vector = vector / np.linalg.norm(vector) * np.radians(degrees)
    return cv2.Rodrigues(vector)[0]


def default_config():
    return settings.load_config(settings.DEFAULT_CONFIG)


def write_tcp(directory, *, matrix=None, roll=-0.079692433, gripper=-0.0043705, tolerances=None, **extra):
    if matrix is None:
        matrix = np.eye(4)
        matrix[:3, 3] = [0.01, -0.035, 0.06]
    matrix = np.asarray(matrix, float)
    data = {'marker_id': 8, 'matrix': matrix.tolist(),
            'required_joint_positions': {'limb5_to_limb4_joint': roll, 'gripper_left_joint': gripper},
            'tolerances': tolerances or {'limb5_to_limb4_joint': 0.01, 'gripper_left_joint': 0.0015},
            'provenance': {'kind': 'unit-test fixture, not a calibration'}}
    data.update(extra)
    path = Path(directory) / 'marker8_to_tcp.json'
    path.write_text(json.dumps(data))
    return path


def small_config(targets=(0, 1), valid_frames=3):
    config = copy.deepcopy(default_config())
    config.update(target_ids=list(targets), valid_frames=valid_frames, min_valid_frames=valid_frames,
                  settling_s=0.0, capture_timeout_s=2.0,
                  cycles=2)
    config['robot_camera']['pair_every_n_external_frames'] = 2
    config['ball']['crosscheck_every_n_valid_frames'] = 0
    config['storage']['min_free_gb'] = 0
    return config


def read_csv(path):
    import csv
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def tempdir(test):
    directory = tempfile.TemporaryDirectory()
    test.addCleanup(directory.cleanup)
    return Path(directory.name)
