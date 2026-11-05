"""Read-only tests around root-owned metrology.py (not modified here)."""
import unittest

import cv2
import numpy as np

from tests.helpers import LOGITECH_K
import geometry

try:
    import metrology
except ImportError as error:  # scipy missing in this interpreter
    metrology = None
    REASON = str(error)


def render_ball(center, radius, k, shape=(1080, 1920), supersample=4):
    """Anti-aliased orange sphere silhouette on a grey background (exact tangent cone)."""
    h, w = shape
    s = supersample
    distance = np.linalg.norm(center)
    axis = center / distance
    half_angle = np.arcsin(radius / distance)
    uv = (k @ center)[:2] / center[2]
    pad = int(k[0, 0] * radius / center[2] * 1.6) + 4
    x0, x1 = int(uv[0]) - pad, int(uv[0]) + pad
    y0, y1 = int(uv[1]) - pad, int(uv[1]) + pad
    xs = (np.arange(x0 * s, x1 * s) + 0.5) / s - 0.5
    ys = (np.arange(y0 * s, y1 * s) + 0.5) / s - 0.5
    gx, gy = np.meshgrid(xs, ys)
    rays = np.stack([(gx - k[0, 2]) / k[0, 0], (gy - k[1, 2]) / k[1, 1], np.ones_like(gx)], -1)
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    inside = (rays @ axis) > np.cos(half_angle)
    coverage = cv2.resize(inside.astype(np.float32), (x1 - x0, y1 - y0), interpolation=cv2.INTER_AREA)
    image = np.full((h, w, 3), 128, np.float32)
    orange = np.array([0, 140, 255], np.float32)
    patch = image[y0:y1, x0:x1]
    image[y0:y1, x0:x1] = patch * (1 - coverage[..., None]) + orange * coverage[..., None]
    return image.astype(np.uint8)


@unittest.skipIf(metrology is None, 'metrology unavailable')
class SphereTests(unittest.TestCase):
    def test_fit_sphere_rays_exact_points(self):
        center = np.array([0.03, -0.02, 0.45])
        radius = 0.02
        axis = center / np.linalg.norm(center)
        basis = np.cross(axis, [0, 1, 0])
        basis /= np.linalg.norm(basis)
        second = np.cross(axis, basis)
        angle = np.arcsin(radius / np.linalg.norm(center))
        phi = np.linspace(0, 2 * np.pi, 200, endpoint=False)
        rays = (np.cos(angle) * axis[None] + np.sin(angle) * (np.cos(phi)[:, None] * basis + np.sin(phi)[:, None] * second))
        pixels = (LOGITECH_K @ rays.T).T
        pixels = pixels[:, :2] / pixels[:, 2:]
        fitted, quality, _ = metrology.fit_sphere_rays(pixels, LOGITECH_K, np.zeros(5), radius)
        self.assertLess(np.linalg.norm(fitted - center) * 1000, 0.05)
        self.assertGreater(quality['arc_degrees'], 350)

    def test_orange_sphere_on_rendered_image(self):
        center = np.array([0.02, 0.01, 0.50])
        image = render_ball(center, 0.02, LOGITECH_K)
        marker_pose = geometry.rigid(np.diag([1.0, -1.0, -1.0]), [0.0, 0.0, 0.5])
        side = 0.015
        corners3 = np.array([[-side, side, 0], [side, side, 0], [side, -side, 0], [-side, -side, 0]])
        corners = (LOGITECH_K @ (marker_pose[:3, :3] @ corners3.T + marker_pose[:3, 3:4])).T
        corners = corners[:, :2] / corners[:, 2:]
        found, quality, _, _ = metrology.orange_sphere(image, LOGITECH_K, np.zeros(5), marker_pose, corners)
        error_mm = np.linalg.norm(found - center) * 1000
        # Depth along the viewing ray is the weak direction of a silhouette fit.
        self.assertLess(error_mm, 3.0, quality)


if __name__ == '__main__':
    unittest.main()
