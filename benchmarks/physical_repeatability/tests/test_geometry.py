import unittest

import cv2
import numpy as np

from tests import helpers
from tests.helpers import LOGITECH_K, rot
import geometry
import vision


def project(points, pose, k, d):
    rv = cv2.Rodrigues(pose[:3, :3])[0]
    uv, _ = cv2.projectPoints(points, rv, pose[:3, 3], k, d)
    return uv.reshape(-1, 2)


def marker_image(marker_id, pixels):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


class TransformTests(unittest.TestCase):
    def setUp(self):
        self.config = helpers.default_config()
        self.detector = vision.Markers(self.config)
        self.d = np.zeros(5)
        self.camera_target = geometry.rigid(rot([1, 0.2, 0], 160), [0.05, 0.08, 0.55])
        self.camera_marker = geometry.rigid(rot([0.9, -0.3, 0.1], 150), [-0.06, -0.04, 0.50])

    def pose_from_projection(self, marker_id, truth):
        side = self.config['marker_sizes_mm'][str(marker_id)] / 1000
        corners = project(vision.square_points(side), truth, LOGITECH_K, self.d)
        return self.detector.pose(marker_id, corners, LOGITECH_K, self.d)[0]

    def test_target_marker_direction_and_sizes(self):
        ct = self.pose_from_projection(0, self.camera_target)   # 40 mm
        cm = self.pose_from_projection(8, self.camera_marker)   # 30 mm
        np.testing.assert_allclose(ct, self.camera_target, atol=1e-6)
        np.testing.assert_allclose(cm, self.camera_marker, atol=1e-6)
        expected = np.linalg.inv(self.camera_target) @ self.camera_marker
        result = geometry.target_marker(ct, cm)
        np.testing.assert_allclose(result, expected, atol=1e-6)
        wrong_direction = np.linalg.inv(self.camera_marker) @ self.camera_target
        self.assertGreater(np.linalg.norm(result[:3, 3] - wrong_direction[:3, 3]), 0.01)

    def test_wrong_marker_size_scales_distance(self):
        config = dict(self.config, marker_sizes_mm=dict(self.config['marker_sizes_mm'], **{'8': 40}))
        corners = project(vision.square_points(0.030), self.camera_marker, LOGITECH_K, self.d)
        wrong = vision.Markers(config).pose(8, corners, LOGITECH_K, self.d)[0]
        self.assertAlmostEqual(wrong[2, 3] / self.camera_marker[2, 3], 40 / 30, places=4)

    def test_tcp_offset_is_in_marker_frame(self):
        target_marker = geometry.target_marker(self.camera_target, self.camera_marker)
        offset = np.eye(4)
        offset[:3, 3] = [0.01, -0.035, 0.06]
        tcp = geometry.target_tcp(target_marker, offset)
        np.testing.assert_allclose(tcp[:3, 3], target_marker[:3, 3] + target_marker[:3, :3] @ offset[:3, 3])
        ball_camera = (self.camera_marker @ np.r_[offset[:3, 3], 1])[:3]
        np.testing.assert_allclose(geometry.point_in_target(self.camera_target, ball_camera), tcp[:3, 3], atol=1e-12)
        np.testing.assert_allclose(tcp[:3, :3], target_marker[:3, :3])

    def test_invert_and_rigid_checks(self):
        np.testing.assert_allclose(geometry.invert(self.camera_target) @ self.camera_target, np.eye(4), atol=1e-12)
        self.assertTrue(geometry.is_rigid(self.camera_target))
        bad = self.camera_target.copy()
        bad[:3, :3] *= 1.01
        self.assertFalse(geometry.is_rigid(bad))

    def test_rendered_aruco_end_to_end(self):
        """Render markers 0 (40 mm) and 8 (30 mm), detect with the production detector."""
        image = np.full((1080, 1920, 3), 255, np.uint8)
        for marker_id, truth in ((0, geometry.rigid(rot([1, 0, 0], 165), [0.05, 0.03, 0.5])),
                                 (8, geometry.rigid(rot([1, 0.3, 0], 155), [-0.07, -0.02, 0.45]))):
            side = self.config['marker_sizes_mm'][str(marker_id)] / 1000
            corners = project(vision.square_points(side), truth, LOGITECH_K, self.d)
            source = marker_image(marker_id, 600)
            src = np.float32([[0, 0], [600, 0], [600, 600], [0, 600]])
            homography = cv2.getPerspectiveTransform(src, np.float32(corners))
            warped = cv2.warpPerspective(cv2.cvtColor(source, cv2.COLOR_GRAY2BGR), homography, (1920, 1080),
                                         flags=cv2.INTER_AREA, borderValue=(255, 255, 255))
            mask = cv2.warpPerspective(np.full((600, 600), 255, np.uint8), homography, (1920, 1080))
            image[mask > 0] = warped[mask > 0]
            setattr(self, f'truth{marker_id}', truth)
        found = self.detector.detect(image)
        self.assertIn(0, found)
        self.assertIn(8, found)
        ct = self.detector.pose(0, found[0], LOGITECH_K, self.d)[0]
        cm = self.detector.pose(8, found[8], LOGITECH_K, self.d)[0]
        expected = np.linalg.inv(self.truth0) @ self.truth8
        error_mm = np.linalg.norm(geometry.target_marker(ct, cm)[:3, 3] - expected[:3, 3]) * 1000
        self.assertLess(error_mm, 2.0)


class StatisticsTests(unittest.TestCase):
    def test_sample_statistics(self):
        xyz = np.array([[0, 0, 0], [2, 0, 0], [0, 2, 0], [2, 2, 0.0]])
        stats = geometry.repeatability_stats(xyz)
        np.testing.assert_allclose(stats['mean_xyz_mm'], [1, 1, 0])
        np.testing.assert_allclose(stats['std_xyz_mm'], [np.std([0, 2, 0, 2], ddof=1)] * 2 + [0])
        self.assertAlmostEqual(stats['mean_deviation_mm'], np.sqrt(2))
        self.assertAlmostEqual(stats['max_deviation_mm'], np.sqrt(2))
        self.assertAlmostEqual(stats['p95_deviation_mm'], np.sqrt(2))

    def test_single_repetition_has_no_deviation(self):
        stats = geometry.repeatability_stats([[1, 2, 3]])
        self.assertEqual(stats['n'], 1)
        self.assertNotIn('std_xyz_mm', stats)
        self.assertNotIn('mean_deviation_mm', stats)

    def test_median(self):
        transforms = [geometry.rigid(np.eye(3), [x / 1000, 0, 0]) for x in (1, 2, 100)]
        np.testing.assert_allclose(geometry.median_xyz_mm(transforms), [2, 0, 0])


if __name__ == '__main__':
    unittest.main()
