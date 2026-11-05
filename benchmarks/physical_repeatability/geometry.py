"""Rigid-transform helpers and repeatability statistics (numpy only).

Convention: T_a_b maps coordinates expressed in frame b into frame a, so
T_target_marker = inv(T_camera_target) @ T_camera_marker and
T_target_tcp = T_target_marker @ T_marker_tcp.
"""
import numpy as np


def rigid(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation, float).ravel()
    return result


def is_rigid(matrix, atol=1e-6):
    matrix = np.asarray(matrix, float)
    return bool(matrix.shape == (4, 4) and np.all(np.isfinite(matrix))
                and np.allclose(matrix[3], [0, 0, 0, 1], atol=atol)
                and np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=atol)
                and np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=atol))


def invert(matrix):
    matrix = np.asarray(matrix, float)
    rotation = matrix[:3, :3]
    return rigid(rotation.T, -rotation.T @ matrix[:3, 3])


def target_marker(camera_target, camera_marker):
    """Pose of the moving marker expressed in the fixed target frame."""
    return invert(camera_target) @ np.asarray(camera_marker, float)


def target_tcp(target_marker_pose, marker_tcp):
    return np.asarray(target_marker_pose, float) @ np.asarray(marker_tcp, float)


def point_in_target(camera_target, point_camera):
    return (invert(camera_target) @ np.r_[np.asarray(point_camera, float).ravel(), 1.0])[:3]


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def median_xyz_mm(transforms):
    """Component-wise median of translations, in millimetres."""
    values = np.array([np.asarray(t, float)[:3, 3] for t in transforms]) * 1000.0
    return np.median(values, axis=0)


def repeatability_stats(xyz_mm):
    """Sample statistics of repeated positions (n x 3, mm).

    Deviations are Euclidean distances to the centroid. Deviation statistics are
    only reported with at least two repetitions; sample std uses ddof=1.
    """
    xyz = np.asarray(xyz_mm, float).reshape(-1, 3)
    result = {'n': int(len(xyz))}
    if len(xyz) == 0:
        return result
    mean = xyz.mean(axis=0)
    result['mean_xyz_mm'] = mean.tolist()
    if len(xyz) < 2:
        return result
    deviation = np.linalg.norm(xyz - mean, axis=1)
    result.update(std_xyz_mm=xyz.std(axis=0, ddof=1).tolist(),
                  mean_deviation_mm=float(deviation.mean()),
                  max_deviation_mm=float(deviation.max()),
                  p95_deviation_mm=float(np.percentile(deviation, 95)))
    return result
