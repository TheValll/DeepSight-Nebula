"""Direct V4L2 capture, onboard stereo rectification and planar ArUco poses.

Cameras are opened only when Camera(...) is constructed. Optics (focus, zoom,
exposure) are never written; v4l2-ctl is only used read-only to compare the
Logitech controls with the values stored in its intrinsic calibration.
"""
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import cv2
import numpy as np

from errors import CameraFailure
from geometry import rigid
from settings import utc


def square_points(side_m):
    s = side_m / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)


def same_device(a, b):
    if a is None or b is None:
        return False
    return str(a) == str(b) or os.path.realpath(str(a)) == os.path.realpath(str(b))


class Camera:
    """Single owner of one V4L2 device with a continuous draining capture thread.

    Every successful read gets a unique, increasing sequence number and its
    monotonic/UTC acquisition time. Repeated read failures (unplug, device busy)
    put the camera in a sticky failed state: frame() raises CameraFailure
    immediately instead of silently waiting.
    """

    def __init__(self, source, resolution, *, fourcc='MJPG', fps=30, name='camera',
                 max_consecutive_failures=30, buffer_size=2):
        self.source = str(source)
        self.name = name
        self.resolution = tuple(resolution)
        self.error = None
        self.failures = 0
        self.max_consecutive_failures = max_consecutive_failures
        index = int(source) if str(source).isdigit() else str(source)
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise CameraFailure(f'{name}: cannot open {source}; close other owners first')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
        actual = (int(round(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))), int(round(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
        if actual != self.resolution:
            self.cap.release()
            raise CameraFailure(f'{name}: {source} delivers {actual}, expected {self.resolution}')
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._read, name=f'capture-{name}', daemon=True)
        self.thread.start()

    def _read(self):
        while not self.stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None or frame.shape[1::-1] != self.resolution:
                self.failures += 1
                if self.failures >= self.max_consecutive_failures:
                    self.error = f'{self.failures} consecutive read failures on {self.source}'
                    return
                time.sleep(0.02)
                continue
            self.failures = 0
            with self.lock:
                self.sequence += 1
                self.latest = (self.sequence, time.monotonic(), utc(), frame)

    def frame(self, after=0, timeout=2.0, max_age_s=1.0):
        """Return (sequence, monotonic, utc, image) newer than `after`."""
        end = time.monotonic() + timeout
        while True:
            if self.error:
                raise CameraFailure(f'{self.name}: {self.error}')
            with self.lock:
                data = self.latest
            if data and data[0] > after and time.monotonic() - data[1] < max_age_s:
                return data
            if time.monotonic() >= end:
                raise CameraFailure(f'{self.name}: no new frame within {timeout:.1f} s')
            time.sleep(0.003)

    def healthy(self):
        return self.error is None and self.thread.is_alive()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.cap.release()


class StereoRectifier:
    """Python port of the left-eye rectification in rectification_node.cpp (alpha 0)."""

    def __init__(self, path, eye_size=(1280, 720)):
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise ValueError(f'cannot read stereo calibration {path}')
        values = {k: storage.getNode(k).mat() for k in ('K1', 'D1', 'K2', 'D2', 'R', 'T')}
        storage.release()
        if any(v is None for v in values.values()):
            raise ValueError(f'incomplete stereo calibration {path}')
        r1, _, p1, _, _, _, _ = cv2.stereoRectify(values['K1'], values['D1'], values['K2'], values['D2'],
                                                  eye_size, values['R'], values['T'],
                                                  flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0,
                                                  newImageSize=eye_size)
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(values['K1'], values['D1'], r1, p1,
                                                             eye_size, cv2.CV_32FC1)
        self.eye_size = eye_size
        self.k = p1[:3, :3].copy()
        self.distortion = np.zeros(5)

    def left(self, raw):
        width = self.eye_size[0]
        if raw.shape[1] != 2 * width or raw.shape[0] != self.eye_size[1]:
            raise CameraFailure(f'unexpected stereo frame {raw.shape}')
        return cv2.remap(raw[:, :width], self.map_x, self.map_y, cv2.INTER_LINEAR)


class Markers:
    def __init__(self, config):
        self.config = config
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, config['dictionary']))
        self.modern = hasattr(aruco, 'ArucoDetector')
        self.detector = self._make(config.get('detector_parameters', {}))
        self.fallback = (self._make(config['fallback_detector_parameters'])
                         if config.get('fallback_detector_parameters') else None)
        self.expected = set(config['target_ids']) | {config['gripper_id']}

    def _make(self, values):
        aruco = cv2.aruco
        params = aruco.DetectorParameters() if self.modern else aruco.DetectorParameters_create()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        for name, value in values.items():
            setattr(params, name, value)
        return aruco.ArucoDetector(self.dictionary, params) if self.modern else params

    def _run(self, detector, image):
        if self.modern:
            return detector.detectMarkers(image)
        return cv2.aruco.detectMarkers(image, self.dictionary, parameters=detector)

    def detect(self, image):
        """{id: 4x2 corners}. Duplicate IDs or conflicting windows raise ValueError."""
        corners, ids, _ = self._run(self.detector, image)
        if ids is not None and len(set(ids.ravel())) != len(ids):
            raise ValueError('duplicate marker IDs in one image')
        found = {} if ids is None else {int(i): c.reshape(4, 2).astype(np.float64) for i, c in zip(ids.ravel(), corners)}
        if self.fallback is not None and self.expected - set(found):
            other, other_ids, _ = self._run(self.fallback, image)
            if other_ids is not None:
                if len(set(other_ids.ravel())) != len(other_ids):
                    raise ValueError('duplicate fallback marker IDs')
                for marker, corner in zip(other_ids.ravel(), other):
                    marker = int(marker)
                    if marker not in self.expected:
                        continue
                    corner = corner.reshape(4, 2).astype(np.float64)
                    if marker in found:
                        if np.sqrt(np.mean((found[marker] - corner)**2)) > 10:
                            raise ValueError('conflicting marker locations across detection windows')
                    else:
                        found[marker] = corner
        return found

    def pose(self, marker_id, corners, k, distortion):
        """IPPE-square pose of one marker (T_camera_marker, reprojection RMS px)."""
        edges = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
        if min(edges) < self.config['min_marker_edge_px']:
            raise ValueError('marker too small or too oblique')
        points = square_points(self.config['marker_sizes_mm'][str(marker_id)] / 1000)
        result = cv2.solvePnPGeneric(points, corners, k, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not result[0]:
            raise ValueError('PnP failed')
        candidates = []
        for rv, tv in zip(result[1], result[2]):
            rotation = cv2.Rodrigues(rv)[0]
            if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(tv)):
                continue
            if np.min((rotation @ points.T + tv).T[:, 2]) <= 0:
                continue
            uv, _ = cv2.projectPoints(points, rv, tv, k, distortion)
            error = float(np.sqrt(np.mean(np.sum((uv.reshape(4, 2) - corners)**2, axis=1))))
            candidates.append((error, rigid(rotation, tv)))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.config['max_reprojection_error_px']:
            raise ValueError('reprojection error too high')
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < self.config['min_pose_solution_error_gap_px']:
            raise ValueError('ambiguous planar pose')
        return candidates[0][1], candidates[0][0]


def load_intrinsics(path, frame_size, source=None):
    data = json.loads(Path(path).read_text())
    if list(data['resolution']) != list(frame_size):
        raise ValueError(f'camera resolution {frame_size} differs from calibration {data["resolution"]}')
    if source is not None and not same_device(data.get('camera_source'), source):
        raise ValueError(f'camera source {source} differs from calibrated {data.get("camera_source")}')
    k = np.asarray(data['camera_matrix'], np.float64)
    d = np.asarray(data['distortion'], np.float64)
    if k.shape != (3, 3) or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('invalid camera intrinsics')
    return k, d, data


def read_controls(source, names=('focus_automatic_continuous', 'focus_absolute', 'zoom_absolute')):
    """Read-only v4l2-ctl query; never sets a control."""
    result = subprocess.run(['v4l2-ctl', '-d', str(source), '--get-ctrl=' + ','.join(names)],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise CameraFailure('cannot read external camera controls: ' + result.stderr.strip())
    return {line.split(':')[0].strip(): int(line.split(':')[1]) for line in result.stdout.splitlines() if ':' in line}


def check_controls(controls, calibration_data):
    if controls.get('focus_automatic_continuous') != 0:
        raise ValueError('Logitech autofocus is enabled; disable it manually (not changed automatically)')
    if controls != calibration_data.get('camera_controls'):
        raise ValueError(f'external camera controls {controls} differ from calibration '
                         f'{calibration_data.get("camera_controls")}; restore them manually')
