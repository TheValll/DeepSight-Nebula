#!/usr/bin/env python3
"""Correct the camera tilt and height from a checkerboard lying on the desk.

The printed camera support can move when the robot is stored, so its pose in
the URDF drifts. At startup this node looks for the checkerboard in the left
rectified image, measures the desk plane with PnP and compares it with the
desk plane predicted by the robot model (z = desk_height in base_link).

The smallest correction that makes both planes coincide (tilt + height along
the desk normal) is published as a static transform:

    camera_left_optical_frame (URDF) -> camera_left_measured_optical_frame

The stereo images and detections use the measured frame. Horizontal position
and yaw cannot be observed from a plane and stay as in the URDF.

Board not seen (absent or out of view): the calibration is skipped and the last
saved correction is kept (identity if none was saved).
"""
from datetime import datetime
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

URDF_FRAME = 'camera_left_optical_frame'
MEASURED_FRAME = 'camera_left_measured_optical_frame'


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    rotation_vector, _ = cv2.Rodrigues(matrix)
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    axis = rotation_vector.ravel() / angle
    s = math.sin(angle / 2)
    return axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)


def oriented_plane(normal, distance):
    """Plane n.p = d with the normal pointing from the camera to the plane (d > 0)."""
    normal = np.asarray(normal, dtype=float)
    norm = np.linalg.norm(normal)
    normal, distance = normal / norm, distance / norm
    return (normal, distance) if distance > 0 else (-normal, -distance)


def plane_correction(measured, predicted):
    """Smallest rigid correction mapping the measured desk plane onto the predicted one.

    Returns (R, t) with p_urdf = R p_measured + t.
    """
    (n_m, d_m), (n_u, d_u) = measured, predicted
    axis = np.cross(n_m, n_u)
    sin_angle, cos_angle = np.linalg.norm(axis), float(np.clip(n_m @ n_u, -1.0, 1.0))
    if sin_angle < 1e-12:
        rotation = np.eye(3)
    else:
        rotation, _ = cv2.Rodrigues(axis / sin_angle * math.atan2(sin_angle, cos_angle))
    return rotation, (d_u - d_m) * n_u


class CameraDeskCalibration(Node):
    def __init__(self) -> None:
        super().__init__('camera_desk_calibration')
        self.correction_file = Path(self.required('correction_file', Parameter.Type.STRING))
        self.calibrate = self.required('calibrate', Parameter.Type.BOOL)
        self.base_frame = self.required('base_frame', Parameter.Type.STRING)
        self.desk_height = self.required('desk_height', Parameter.Type.DOUBLE)
        self.board_thickness = self.required('board_thickness', Parameter.Type.DOUBLE)
        columns = self.required('board_inner_columns', Parameter.Type.INTEGER)
        rows = self.required('board_inner_rows', Parameter.Type.INTEGER)
        square = self.required('board_square_size', Parameter.Type.DOUBLE)
        self.timeout = self.required('search_timeout', Parameter.Type.DOUBLE)
        self.samples_needed = self.required('samples', Parameter.Type.INTEGER)
        self.max_tilt = math.radians(self.required('max_tilt_correction_deg', Parameter.Type.DOUBLE))
        self.max_height = self.required('max_height_correction', Parameter.Type.DOUBLE)

        self.pattern = (columns, rows)
        grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square
        self.board_points = np.hstack([grid, np.zeros((len(grid), 1))]).astype(np.float32)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Not tf2_ros.StaticTransformBroadcaster: it never replaces a transform
        # already sent for the same child frame, so the correction computed after
        # the initial one would be ignored. Same QoS: the last message is latched.
        self.static_publisher = self.create_publisher(TFMessage, '/tf_static', QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.camera_matrix = None
        self.measured_planes = []
        self.predicted_planes = []
        self.searching = self.calibrate
        self.start = self.get_clock().now()

        # Publish the last known correction at once, so the TF chain exists
        # before the stereo detections start.
        rotation, translation, origin = self.load_correction()
        self.publish(rotation, translation)
        self.get_logger().info(f'Correction initiale : {origin}.')

        if self.calibrate:
            self.create_subscription(CameraInfo, '/stereo/depth/camera_info',
                                     self.store_camera_info, qos_profile_sensor_data)
            self.image_subscription = self.create_subscription(
                Image, '/stereo/left/image_rect', self.process_image, qos_profile_sensor_data)
            self.create_timer(0.5, self.check_timeout)
            self.get_logger().info(
                f'Recherche du damier {columns}x{rows} sur le bureau '
                f'pendant {self.timeout:.0f} s...')

    def required(self, name, kind):
        value = self.declare_parameter(name, kind).value
        if value is None:
            raise ValueError(f'Missing ROS parameter {name!r} (see config/pipeline.yaml)')
        return value

    def load_correction(self):
        try:
            data = yaml.safe_load(self.correction_file.read_text())
            rotation = quaternion_to_matrix(*data['rotation_xyzw'])
            translation = np.array(data['translation_m'], dtype=float)
            return rotation, translation, f"enregistrée le {data['date']} ({self.correction_file})"
        except FileNotFoundError:
            return np.eye(3), np.zeros(3), 'aucune (URDF seul, pas encore de calibration)'
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().warning(f'Fichier de correction illisible, ignoré : {error}')
            return np.eye(3), np.zeros(3), 'aucune (fichier illisible)'

    def publish(self, rotation, translation):
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = URDF_FRAME
        message.child_frame_id = MEASURED_FRAME
        message.transform.translation.x, message.transform.translation.y, \
            message.transform.translation.z = map(float, translation)
        (message.transform.rotation.x, message.transform.rotation.y,
         message.transform.rotation.z, message.transform.rotation.w) = map(
            float, matrix_to_quaternion(rotation))
        self.static_publisher.publish(TFMessage(transforms=[message]))

    def store_camera_info(self, message):
        # The depth camera_info is at half resolution; the left image is full size.
        fx, fy, cx, cy = message.k[0], message.k[4], message.k[2], message.k[5]
        self.camera_matrix = np.array([[2 * fx, 0, 2 * cx + 0.5],
                                       [0, 2 * fy, 2 * cy + 0.5],
                                       [0, 0, 1]])

    def process_image(self, message):
        if not self.searching or self.camera_matrix is None:
            return
        stamp = Time.from_msg(message.header.stamp)
        if not self.tf_buffer.can_transform(self.base_frame, URDF_FRAME, stamp):
            return
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, 'bgr8'), cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, self.pattern)
        if not found:
            return
        ok, rotation_vector, translation = cv2.solvePnP(
            self.board_points, corners, self.camera_matrix, None)
        if not ok:
            return
        board_rotation, _ = cv2.Rodrigues(rotation_vector)
        normal = board_rotation[:, 2]
        # Board surface, then the desk below it.
        n_m, d_m = oriented_plane(normal, float(normal @ translation.ravel()))
        measured = (n_m, d_m + self.board_thickness)

        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, URDF_FRAME, stamp)
        except TransformException:
            return
        q = transform.transform.rotation
        base_from_camera = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        camera_z = transform.transform.translation.z
        # Desk z = desk_height in base_link, expressed in the URDF camera frame.
        predicted = oriented_plane(base_from_camera.T @ np.array([0.0, 0.0, 1.0]),
                                   self.desk_height - camera_z)
        self.measured_planes.append(measured)
        self.predicted_planes.append(predicted)
        if len(self.measured_planes) >= self.samples_needed:
            self.finish()

    def check_timeout(self):
        if not self.searching:
            return
        if self.get_clock().now() - self.start > Duration(seconds=self.timeout):
            self.searching = False
            self.get_logger().warning(
                f'Damier non vu ({len(self.measured_planes)} image(s) valide(s) sur '
                f'{self.samples_needed}) : calibration ignorée, correction précédente conservée.')

    def finish(self):
        self.searching = False
        def mean(planes):
            normal = np.mean([n for n, _ in planes], axis=0)
            return normal / np.linalg.norm(normal), float(np.median([d for _, d in planes]))
        measured, predicted = mean(self.measured_planes), mean(self.predicted_planes)
        rotation, translation = plane_correction(measured, predicted)
        tilt = math.acos(float(np.clip(measured[0] @ predicted[0], -1.0, 1.0)))
        height = predicted[1] - measured[1]
        spread = np.std([d for _, d in self.measured_planes]) * 1000
        summary = (f'inclinaison {math.degrees(tilt):.2f} deg, hauteur {height * 1000:+.1f} mm '
                   f'(dispersion {spread:.1f} mm sur {len(self.measured_planes)} images)')
        if tilt > self.max_tilt or abs(height) > self.max_height:
            self.get_logger().error(
                f'Correction refusée, trop grande : {summary}. Vérifier que le damier est '
                'bien à plat sur le bureau. Correction précédente conservée.')
            return
        self.publish(rotation, translation)
        self.correction_file.parent.mkdir(parents=True, exist_ok=True)
        self.correction_file.write_text(yaml.safe_dump({
            'date': datetime.now().isoformat(timespec='seconds'),
            'parent_frame': URDF_FRAME,
            'child_frame': MEASURED_FRAME,
            'rotation_xyzw': [float(v) for v in matrix_to_quaternion(rotation)],
            'translation_m': [float(v) for v in translation],
            'tilt_deg': round(math.degrees(tilt), 3),
            'height_mm': round(height * 1000, 2),
        }, sort_keys=False))
        self.get_logger().info(f'Caméra recalée sur le bureau : {summary}. '
                               f'Enregistré dans {self.correction_file}.')
        self.destroy_subscription(self.image_subscription)


def main() -> None:
    rclpy.init()
    node = CameraDeskCalibration()
    try:
        # Stay alive: the static transform is served to late subscribers.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
