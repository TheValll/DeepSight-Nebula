"""Visit sequencing with fake robot/cameras/detector and a real on-disk recorder."""
import csv
import json
import sqlite3
import threading
import time
import unittest

import numpy as np

from tests import helpers
from tests.helpers import rot
import benchmark
import errors as E
import geometry
import report
import settings
import storage

ROLL, GRIPPER = -0.079692433, -0.0043705


class FakeCamera:
    def __init__(self, shape=(48, 64, 3)):
        self.sequence = 0
        self.shape = shape
        self.dead = False

    def frame(self, after=0, timeout=2.0, max_age_s=1.0):
        if self.dead:
            raise E.CameraFailure('fake camera unplugged')
        if getattr(self, 'stalled', False):
            time.sleep(min(timeout, 0.5))
            raise E.CameraFailure('fake camera: no new frame')
        time.sleep(0.002)
        self.sequence = max(self.sequence, after) + 1
        image = np.random.default_rng(self.sequence).integers(0, 255, self.shape, dtype=np.uint8)
        return self.sequence, time.monotonic(), settings.utc(), image

    def healthy(self):
        return not self.dead


class FakeRectifier:
    def left(self, raw):
        return raw[:, : raw.shape[1] // 2].copy()


class FakeDetector:
    """Returns ground-truth poses with small noise; can hide markers per target."""

    def __init__(self, hide_target=()):
        self.hide_target = set(hide_target)
        self.rng = np.random.default_rng(1)
        self.current = None

    def detect(self, image):
        found = {8: np.zeros((4, 2))}
        for target in range(8):
            if target not in self.hide_target:
                found[target] = np.full((4, 2), float(target))
        return found

    def pose(self, marker_id, corners, k, d):
        if marker_id == 8:
            noise = self.rng.normal(0, 0.0002, 3)
            return geometry.rigid(rot([1, 0, 0], 150), np.array([-0.05, 0.0, 0.5]) + noise), 0.2
        return geometry.rigid(rot([1, 0, 0], 170), [0.04 * marker_id, 0.05, 0.55]), 0.2


class FakeRobot:
    def __init__(self, test, session, fail_on=None, drift_after=None):
        self.test = test
        self.session = session
        self.home = {j: 0.0 for j in settings.ARM_JOINTS}
        self.home[settings.ROLL_JOINT] = ROLL
        self.calls = []
        self.fail_on = fail_on
        self.drift_after = drift_after
        self.state = dict(self.home, **{settings.GRIPPER_JOINT: GRIPPER})

    def joints(self):
        state = dict(self.state)
        if self.drift_after is not None and len(self.calls) >= self.drift_after:
            state[settings.GRIPPER_JOINT] = GRIPPER + 0.01   # ball slipping / gripper opening
        return state, 0.01

    def _status(self, visit_id):
        connection = sqlite3.connect(self.session / storage.DB_NAME)
        try:
            row = connection.execute("SELECT status FROM visits WHERE json_extract(data, '$.visit_id')=?",
                                     (visit_id,)).fetchone()
        finally:
            connection.close()
        return row and row[0]

    def go(self, joints, label=''):
        visit_id, leg = label.split(' ')
        # Durability ordering: visit ACK before outbound; measurement persisted before home.
        self.test.assertEqual(self._status(visit_id), 'started' if leg == 'outbound' else 'measured')
        self.calls.append(label)
        if self.fail_on == label:
            error = E.MotionFailure(f'{label}: fake controller abort')
            error.log = {'commanded_joints': joints}
            raise error
        self.state.update(joints)
        return {'commanded_joints': joints, 'max_abs_error_rad': 0.001, 'errors_rad': {},
                'transit_roll_excursion_rad': 0.0}


class FlowTests(unittest.TestCase):
    def setUp(self):
        self.base = helpers.tempdir(self)
        self.session = settings.new_session_dir('test', base=self.base)
        self.config = helpers.small_config()
        self.guard = settings.load_tcp_calibration(helpers.write_tcp(self.base))
        self.config_path = self.base / 'config.json'
        self.config_path.write_text(json.dumps(self.config))
        self.paths = settings.fingerprint_paths(dict(self.config, tcp_calibration=self.guard['path']),
                                                self.config_path)
        self.recorder = storage.Recorder(self.session, min_free_bytes=0)
        self.addCleanup(self._close)
        self.logs = []

    def _close(self):
        try:
            self.recorder.close()
        except E.PersistenceFailure:
            pass

    def make(self, robot=None, detector=None, stop=None, onboard=True):
        self.robot = robot or FakeRobot(self, self.session)
        return benchmark.Benchmark(
            self.config, robot=self.robot, external=FakeCamera(), onboard=FakeCamera((24, 64, 3)) if onboard else None,
            rectifier=FakeRectifier(), detector=detector or FakeDetector(), recorder=self.recorder,
            k=np.eye(3), distortion=np.zeros(5), guard=self.guard, fingerprint_paths=self.paths,
            fingerprint=settings.fingerprint(self.paths), stop_event=stop, log=self.logs.append)

    def visits(self):
        return {r['visit_id']: r for r in helpers.read_csv((self.session / 'measurements.csv'))}

    def test_resume_skips_recorded_cycles_and_first_targets(self):
        self.config.update(cycles=3, start_cycle=2, start_target=1)
        bench = self.make(onboard=False)
        bench.run()
        self.assertEqual(list(self.visits()), ['c002_t1', 'c003_t0', 'c003_t1'])
        self.assertEqual(self.robot.calls, ['c002_t1 outbound', 'c002_t1 home',
                                          'c003_t0 outbound', 'c003_t0 home',
                                          'c003_t1 outbound', 'c003_t1 home'])

    def test_minimum_frames_accepts_10_and_16_but_rejects_9(self):
        self.config.update(valid_frames=20, min_valid_frames=10)
        for limit in (9, 10, 16):
            with self.subTest(valid_frames=limit):
                bench = self.make(onboard=False)
                camera = bench.external
                original = camera.frame

                def finite_frames(*args, **kwargs):
                    if camera.sequence >= limit:
                        camera.dead = True
                    return original(*args, **kwargs)

                camera.frame = finite_frames
                result = bench.visit(limit, 0)
                self.assertEqual(result['measurement_valid'], limit >= 10)
                self.assertEqual(result['valid_frames'], limit)
                self.assertEqual(result['required_frames'], 10)
                self.assertEqual(result['requested_frames'], 20)
                self.assertFalse(result['frame_target_reached'])
                self.assertTrue(result['home_success'])

    def test_complete_run(self):
        bench = self.make()
        bench.run()
        self.recorder.close()
        rows = self.visits()
        self.assertEqual(list(rows), ['c001_t0', 'c001_t1', 'c002_t0', 'c002_t1'])
        self.assertTrue(all(r['status'] == 'complete' for r in rows.values()))
        self.assertEqual(self.robot.calls, ['c001_t0 outbound', 'c001_t0 home', 'c001_t1 outbound', 'c001_t1 home',
                                            'c002_t0 outbound', 'c002_t0 home', 'c002_t1 outbound', 'c002_t1 home'])
        detector = FakeDetector()
        expected_marker = geometry.target_marker(detector.pose(0, None, None, None)[0],
                                                 geometry.rigid(rot([1, 0, 0], 150), [-0.05, 0, 0.5]))
        expected_tcp = geometry.target_tcp(expected_marker, self.guard['matrix'])
        row = rows['c001_t0']
        np.testing.assert_allclose([float(row[f'marker_{a}_mm']) for a in 'xyz'], expected_marker[:3, 3] * 1000, atol=1)
        np.testing.assert_allclose([float(row[f'tcp_{a}_mm']) for a in 'xyz'], expected_tcp[:3, 3] * 1000, atol=1)
        self.assertEqual(row['valid_frames'], '3')
        self.assertEqual(row['robot_target_visible'], 'True')
        frames = [json.loads(line) for line in (self.session / 'frames.jsonl').read_text().splitlines()]
        external = [f for f in frames if f['camera'] == 'external']
        onboard = [f for f in frames if f['camera'] == 'onboard']
        self.assertEqual(len(external), 12)
        self.assertGreaterEqual(len(onboard), 8)
        self.assertTrue(all('T_target_tcp' in f and 'T_camera_marker' in f and 'joints' in f for f in external))
        self.assertTrue(all({'raw_stereo', 'rectified_left'} <= set(f['images']) for f in onboard))
        self.assertEqual(len(list(helpers.read_csv((self.session / 'cycles.csv')))), 2)
        result = report.analyse(*report.load(self.session)[:3], self.config)
        self.assertEqual(result['per_target']['0']['valid_repetitions'], 2)
        self.assertIn('p95_deviation_mm', result['per_target']['0']['tcp'])

    def test_vision_failure_is_recorded_and_run_continues(self):
        self.make(detector=FakeDetector(hide_target={1})).run()
        rows = self.visits()
        failed = rows['c001_t1']
        self.assertEqual((failed['status'], failed['failure_category']), ('failed', E.ARUCO_TARGET_NOT_DETECTED))
        self.assertEqual((failed['home_success'], failed['measurement_valid']), ('True', 'False'))
        self.assertEqual(failed['tcp_x_mm'], '')
        self.assertEqual(rows['c002_t0']['status'], 'complete')
        frames = [json.loads(line) for line in (self.session / 'frames.jsonl').read_text().splitlines()]
        rejects = [f for f in frames if f['visit_id'] == 'c001_t1' and f['camera'] == 'external']
        self.assertGreater(len(rejects), 10)    # every rejected attempt is kept, with its image
        self.assertTrue(all(f['images'] for f in rejects))

    def test_motion_failure_stops_without_homing(self):
        robot = FakeRobot(self, self.session, fail_on='c001_t1 outbound')
        with self.assertRaises(E.MotionFailure):
            self.make(robot=robot).run()
        self.assertEqual(robot.calls[-1], 'c001_t1 outbound')
        rows = self.visits()
        self.assertEqual((rows['c001_t1']['status'], rows['c001_t1']['failure_category']), ('failed', 'MOTION_FAILURE'))
        self.assertNotIn('c002_t0', rows)

    def test_transient_stale_feedback_recovers_and_is_recorded(self):
        bench = self.make()
        original = self.robot.joints
        ages = iter([0.53, 0.01])

        def delayed_feedback():
            joints, _ = original()
            return joints, next(ages)

        self.robot.joints = delayed_feedback
        _, age, _ = bench._guard_now('transient delay')
        self.assertEqual(age, 0.01)
        self.assertEqual(self.robot.calls, [])
        self.recorder.barrier()
        events = (self.session / 'events.jsonl').read_text()
        self.assertIn('joint_feedback_wait_started', events)
        self.assertIn('joint_feedback_recovered', events)

    def test_persistent_stale_feedback_stops_without_homing(self):
        self.config['joint_state_recovery_timeout_s'] = 0.02
        bench = self.make()
        original = self.robot.joints

        def lost_feedback():
            joints, age = original()
            return joints, 0.53 if self.robot.calls else age

        self.robot.joints = lost_feedback
        with self.assertRaisesRegex(E.SafetyRejection, 'after waiting'):
            bench.visit(1, 0)
        self.assertEqual(self.robot.calls, ['c001_t0 outbound'])
        self.assertEqual(self.visits()['c001_t0']['failure_category'], 'SAFETY_REJECTION')
        self.assertEqual(self.visits()['c001_t0']['home_success'], 'False')

    def test_stop_interrupts_feedback_recovery(self):
        stop = threading.Event()
        bench = self.make(stop=stop)
        original = self.robot.joints

        def lost_feedback():
            stop.set()
            joints, _ = original()
            return joints, 0.53

        self.robot.joints = lost_feedback
        with self.assertRaises(E.Interrupted):
            bench._guard_now('stop during delay')
        self.assertEqual(self.robot.calls, [])

    def test_gripper_drift_is_a_safety_stop(self):
        robot = FakeRobot(self, self.session, drift_after=1)
        with self.assertRaises(E.SafetyRejection):
            self.make(robot=robot).run()
        self.assertEqual(robot.calls, ['c001_t0 outbound'])
        self.assertEqual(self.visits()['c001_t0']['failure_category'], 'SAFETY_REJECTION')

    def test_stop_request_leaves_robot_in_place(self):
        stop = threading.Event()

        class StoppingDetector(FakeDetector):
            def detect(self, image):
                stop.set()
                return super().detect(image)
        with self.assertRaises(E.Interrupted):
            self.make(detector=StoppingDetector(), stop=stop).run()
        self.assertEqual(self.robot.calls, ['c001_t0 outbound'])
        self.assertEqual(self.visits()['c001_t0']['failure_category'], 'INTERRUPTED')

    def test_fingerprint_change_rejected_before_motion(self):
        bench = self.make()
        bench.visit(1, 0)
        helpers.write_tcp(self.base, roll=-0.0797)   # calibration edited mid-session
        with self.assertRaises(E.SafetyRejection):
            bench.visit(1, 1)
        self.assertEqual(self.robot.calls, ['c001_t0 outbound', 'c001_t0 home'])
        self.assertNotIn('c001_t1', self.visits())
        self.recorder.close()
        events = (self.session / 'events.jsonl').read_text()
        self.assertIn('visit_rejected_before_motion', events)

    def test_persistence_failure_stops_motion(self):
        bench = self.make()
        bench.visit(1, 0)
        self.recorder.error = OSError('disk gone')
        with self.assertRaises(E.PersistenceFailure):
            bench.visit(1, 1)
        self.assertEqual(self.robot.calls, ['c001_t0 outbound', 'c001_t0 home'])

    def test_camera_death_counts_and_stops(self):
        bench = self.make(onboard=False)
        bench.external.stalled = True            # thread alive but no frames
        self.config['max_consecutive_camera_failure_visits'] = 2
        bench.visit(1, 0)
        with self.assertRaises(E.CameraFailure):
            bench.visit(1, 1)
        rows = self.visits()
        self.assertEqual(rows['c001_t0']['failure_category'], E.CAMERA_CAPTURE_FAILURE)
        self.assertEqual(rows['c001_t1']['home_success'], 'True')  # measurement failed, arm returned home
        bench.external.dead = True               # capture thread failed: rejected before motion
        with self.assertRaises(E.CameraFailure):
            bench.visit(2, 0)
        self.assertNotIn('c002_t0', self.visits())


if __name__ == '__main__':
    unittest.main()
