"""Floor geometry and collision exceptions, without hardware access."""
import importlib.util
from pathlib import Path
import unittest

from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene

spec = importlib.util.spec_from_file_location(
    'ground', Path(__file__).parents[1] / 'scripts' / 'add_ground.py')
ground = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ground)


class GroundTests(unittest.TestCase):
    def test_height_and_fixed_base_only(self):
        scene = PlanningScene()
        matrix = scene.allowed_collision_matrix
        matrix.entry_names = ['base_link', 'arm', 'finger']
        matrix.entry_values = [AllowedCollisionEntry(enabled=row) for row in (
            [False, True, False], [True, False, False], [False, False, False])]
        ground.add_floor(scene)
        floor = scene.world.collision_objects[0]
        self.assertEqual(floor.header.frame_id, 'base_link')
        self.assertAlmostEqual(
            floor.primitive_poses[0].position.z + floor.primitives[0].dimensions[2] / 2,
            0.005)
        i = matrix.entry_names.index('ground')
        self.assertEqual(list(matrix.entry_values[i].enabled), [True, False, False, False])
        self.assertTrue(matrix.entry_values[0].enabled[1])
        ground.add_floor(scene)
        self.assertEqual(matrix.entry_names.count('ground'), 1)
        self.assertTrue(scene.is_diff)

    def test_padding_only_on_requested_links_and_floor_stays_real(self):
        scene = PlanningScene()
        ground.add_floor(scene, height=-0.005, clearance=0.0,
                         padded_links=['limb5_link', 'gripper_left_link'], padding=0.012)
        floor = scene.world.collision_objects[0]
        self.assertAlmostEqual(
            floor.primitive_poses[0].position.z + floor.primitives[0].dimensions[2] / 2,
            -0.005)
        self.assertEqual([(p.link_name, p.padding) for p in scene.link_padding],
                         [('limb5_link', 0.012), ('gripper_left_link', 0.012)])

    def test_invalid_padding(self):
        for value in (-0.001, float('nan')):
            with self.assertRaises(ValueError):
                ground.add_floor(PlanningScene(), padded_links=['limb5_link'], padding=value)

    def test_invalid_clearance(self):
        for value in (-0.001, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ground.add_floor(PlanningScene(), clearance=value)


if __name__ == '__main__':
    unittest.main()
