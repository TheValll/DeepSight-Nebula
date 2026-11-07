import contextlib
import io
import json
import sys
import unittest

import numpy as np

from tests import helpers
import benchmark
from errors import SafetyRejection
import settings


class DefaultsTests(unittest.TestCase):
    def test_final_requirements_in_default_config(self):
        config = helpers.default_config()
        self.assertEqual(config['cycles'], 50)
        self.assertEqual(config['velocity_scaling'], 1.0)
        self.assertEqual(config['acceleration_scaling'], 0.1)
        self.assertEqual(config['target_ids'], list(range(8)))
        self.assertEqual(config['gripper_id'], 8)
        self.assertEqual(config['marker_sizes_mm']['8'], 30)
        self.assertTrue(all(config['marker_sizes_mm'][str(i)] == 40 for i in range(8)))
        self.assertEqual(config['valid_frames'], 20)
        self.assertEqual(config['capture_timeout_s'], 8.0)
        self.assertEqual(config['settling_s'], 1.0)
        self.assertEqual(config['joint_state_recovery_timeout_s'], 60.0)
        self.assertFalse(config['orientation_enabled'])
        self.assertTrue(config['robot_camera']['device'].isdigit())
        self.assertTrue(config['external_camera']['device'].isdigit())
        settings.validate_config(config)

    def test_cli_defaults_and_help(self):
        args = benchmark.build_parser().parse_args([])
        self.assertEqual(args.command, 'run')
        self.assertIsNone(args.cycles)
        config = settings.apply_overrides(helpers.default_config(), args)
        self.assertEqual((config['cycles'], config['velocity_scaling']), (50, 1.0))
        out = io.StringIO()
        with contextlib.redirect_stdout(out), self.assertRaises(SystemExit) as stop:
            benchmark.main(['--help'])
        self.assertEqual(stop.exception.code, 0)
        self.assertIn('default: 50', out.getvalue())
        self.assertIn('default: 1.0', out.getvalue())
        self.assertNotIn('robot', [m for m in sys.modules if sys.modules[m] is not None])

    def test_overrides(self):
        args = benchmark.build_parser().parse_args(
            ['--cycles', '2', '--velocity', '0.5', '--external-camera', '9', '--robot-camera', 'none'])
        config = settings.apply_overrides(helpers.default_config(), args)
        self.assertEqual((config['cycles'], config['velocity_scaling'], config['acceleration_scaling']), (2, 0.5, 0.1))
        self.assertEqual(config['external_camera']['device'], '9')
        self.assertIsNone(config['robot_camera']['device'])


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.dir = helpers.tempdir(self)

    def prepare(self, *argv):
        return benchmark.prepare_config(benchmark.build_parser().parse_args(list(argv)))

    def test_long_run_requires_tcp_calibration(self):
        missing = str(self.dir / 'missing.json')
        with self.assertRaisesRegex(ValueError, 'calibration-validation'):
            self.prepare('--tcp-calibration', missing)
        with self.assertRaisesRegex(ValueError, 'exactly 1 cycle'):
            self.prepare('--tcp-calibration', missing, '--calibration-validation')
        config, guard = self.prepare('--tcp-calibration', missing, '--calibration-validation', '--cycles', '1')
        self.assertIsNone(guard)
        with self.assertRaisesRegex(ValueError, 'exactly 1 cycle'):
            self.prepare('--tcp-calibration', str(helpers.write_tcp(self.dir)), '--calibration-validation',
                         '--cycles', '2')

    def test_repository_tcp_calibration_if_present(self):
        """Read-only check of root's real calibration file against the production loader."""
        path = settings.resolve(helpers.default_config()['tcp_calibration'])
        if not path.exists():
            self.skipTest('calibration/marker8_to_tcp.json not provided yet')
        guard = settings.load_tcp_calibration(path)
        settings.check_target_roll_goals(helpers.default_config(), guard)
        config, prepared = self.prepare()
        self.assertEqual(prepared['sha256'], guard['sha256'])
        self.assertEqual(config['cycles'], 50)

    def test_valid_tcp_calibration(self):
        path = helpers.write_tcp(self.dir)
        config, guard = self.prepare('--tcp-calibration', str(path))
        self.assertEqual(config['cycles'], 50)
        np.testing.assert_allclose(guard['matrix'][:3, 3], [0.01, -0.035, 0.06])
        self.assertEqual(guard['sha256'], settings.sha256_file(path))

    def test_rejects_rotation_and_missing_fields(self):
        matrix = np.eye(4)
        matrix[:3, :3] = helpers.rot([0, 0, 1], 10)
        with self.assertRaisesRegex(ValueError, 'identity'):
            settings.load_tcp_calibration(helpers.write_tcp(self.dir, matrix=matrix))
        with self.assertRaisesRegex(ValueError, 'tolerance'):
            settings.load_tcp_calibration(helpers.write_tcp(self.dir, tolerances={'limb5_to_limb4_joint': 0.01}))
        mm = np.eye(4)
        mm[:3, 3] = [10, 35, 60]
        with self.assertRaisesRegex(ValueError, 'metres'):
            settings.load_tcp_calibration(helpers.write_tcp(self.dir, matrix=mm))
        path = helpers.write_tcp(self.dir)
        data = json.loads(path.read_text())
        del data['provenance']
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'provenance'):
            settings.load_tcp_calibration(path)

    def test_roll_goal_and_joint_guard(self):
        guard = settings.load_tcp_calibration(helpers.write_tcp(self.dir, roll=0.3))
        with self.assertRaises(SafetyRejection):
            settings.check_target_roll_goals(helpers.default_config(), guard)
        guard = settings.load_tcp_calibration(helpers.write_tcp(self.dir))
        settings.check_target_roll_goals(helpers.default_config(), guard)
        ok = {'limb5_to_limb4_joint': -0.0797, 'gripper_left_joint': -0.0044}
        settings.check_joint_guard(ok, guard)
        for joint, value in (('limb5_to_limb4_joint', -0.1), ('gripper_left_joint', 0.0)):
            with self.assertRaises(SafetyRejection):
                settings.check_joint_guard(dict(ok, **{joint: value}), guard)
        with self.assertRaises(SafetyRejection):
            settings.check_joint_guard({'limb5_to_limb4_joint': -0.0797}, guard)

    def test_fingerprint_detects_change(self):
        config_path = self.dir / 'config.json'
        config_path.write_text(json.dumps(helpers.default_config()))
        config = helpers.default_config()
        config['tcp_calibration'] = str(helpers.write_tcp(self.dir))
        paths = settings.fingerprint_paths(config, config_path)
        self.assertIn('tcp_calibration', paths)
        self.assertTrue(any(k.startswith('robot_model:') for k in paths))
        expected = settings.fingerprint(paths)
        settings.check_fingerprint(paths, expected)
        helpers.write_tcp(self.dir, roll=-0.08)
        with self.assertRaisesRegex(SafetyRejection, 'tcp_calibration'):
            settings.check_fingerprint(paths, expected)

    def test_invalid_configs(self):
        for change, message in (({'target_ids': [1, 0, 2]}, 'increasing'), ({'gripper_id': 3}, 'gripper'),
                                ({'orientation_enabled': True}, 'orientation'), ({'cycles': 0}, 'cycles'),
                                ({'velocity_scaling': 1.5}, 'velocity')):
            config = dict(helpers.default_config(), **change)
            with self.assertRaisesRegex(ValueError, message):
                settings.validate_config(config)
        config = helpers.default_config()
        config['targets']['3']['validated'] = False
        with self.assertRaisesRegex(ValueError, 'target 3'):
            settings.validate_config(config)


class SessionTests(unittest.TestCase):
    def test_unique_and_no_overwrite(self):
        base = helpers.tempdir(self)
        first = settings.new_session_dir('run', base=base)
        second = settings.new_session_dir('run', base=base)
        self.assertNotEqual(first, second)
        with self.assertRaises(FileExistsError):
            settings.new_session_dir('run', output=first)

    def test_snapshot(self):
        base = helpers.tempdir(self)
        session = settings.new_session_dir('run', base=base)
        config = helpers.default_config()
        config['tcp_calibration'] = str(helpers.write_tcp(base))
        paths = settings.fingerprint_paths(config, settings.DEFAULT_CONFIG)
        manifest = settings.snapshot(session, config, settings.DEFAULT_CONFIG, paths)
        self.assertIn('benchmark.py', manifest['software_sha256'])
        self.assertIn('tcp_calibration', manifest['files'])
        self.assertTrue((session / manifest['files']['tcp_calibration']['copy']).is_file())
        self.assertTrue((session / 'snapshot/effective_config.json').is_file())
        self.assertIn('head', manifest['environment']['git'])

    def test_storage_estimate(self):
        config = helpers.default_config()
        estimate = settings.estimate_storage_bytes(config)
        self.assertGreater(estimate, 10e9)  # 400 visits of lossless 1080p frames
        self.assertLess(estimate, 200e9)


if __name__ == '__main__':
    unittest.main()
