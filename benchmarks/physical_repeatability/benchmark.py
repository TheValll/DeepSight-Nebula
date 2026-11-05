#!/usr/bin/env python3
"""Physical repeatability benchmark (ISO 9283-inspired; no ISO compliance claim).

Each cycle visits fixed ArUco targets 0..7 in increasing order with frozen,
physically validated joint poses, returning to the SRDF reference pose between
targets. At each target the external Logitech camera measures the marker-8 pose
and the operational TCP (centre of the held 40 mm ball, translation-only
calibration) in the target-marker frame. The onboard stereo camera is recorded
for inspection. The ball is a constant payload: no grasp/release is commanded.

Commands (see README):
  run        execute cycles (moves the robot)                       [default]
  preflight  offline checks only; with --live also read joints/cameras and
             plan every leg WITHOUT executing any motion
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

import errors as E
import geometry
import settings
from settings import ARM_JOINTS, GRIPPER_JOINT, ROLL_JOINT, utc


def build_parser():
    defaults = settings.load_config(settings.DEFAULT_CONFIG)
    parser = argparse.ArgumentParser(
        prog='run.sh', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', nargs='?', default='run', choices=['run', 'preflight'])
    parser.add_argument('--config', type=Path, default=settings.DEFAULT_CONFIG,
                        help=f'benchmark configuration (default: {settings.DEFAULT_CONFIG})')
    parser.add_argument('--cycles', type=int, help=f'number of cycles (default: {defaults["cycles"]})')
    parser.add_argument('--start-cycle', type=int, help='resume in a new session at this cycle (inclusive)')
    parser.add_argument('--start-target', type=int, help='first target in the starting cycle; later cycles visit all targets')
    parser.add_argument('--resume-from', type=Path, help='previous session, recorded as provenance; never modified')
    parser.add_argument('--velocity', dest='velocity_scaling', type=float,
                        help=f'MoveIt velocity scaling (default: {defaults["velocity_scaling"]})')
    parser.add_argument('--acceleration', dest='acceleration_scaling', type=float,
                        help=f'MoveIt acceleration scaling (default: {defaults["acceleration_scaling"]})')
    parser.add_argument('--robot-camera', help='onboard stereo V4L2 device, or "none" '
                        f'(default: {defaults["robot_camera"]["device"]})')
    parser.add_argument('--external-camera', help='external V4L2 device; must be the calibrated camera '
                        f'(default: {defaults["external_camera"]["device"]})')
    parser.add_argument('--tcp-calibration', help=f'marker8->TCP JSON (default: {defaults["tcp_calibration"]})')
    parser.add_argument('--calibration-validation', action='store_true',
                        help='exactly 1 cycle; allowed without TCP calibration (marker-8 values only)')
    parser.add_argument('--live', action='store_true',
                        help='preflight only: read joints/controllers/cameras and plan all legs; never executes')
    parser.add_argument('--output', type=Path, help='new session directory (must not exist; default: sessions/<utc>_...)')
    return parser


def prepare_config(args):
    config = settings.apply_overrides(settings.load_config(args.config), args)
    if args.start_cycle is not None:
        config['start_cycle'] = args.start_cycle
    if args.start_target is not None:
        config['start_target'] = args.start_target
    if args.resume_from is not None:
        previous = args.resume_from.resolve()
        if not (previous / 'journal.sqlite3').is_file():
            raise ValueError(f'previous session journal not found: {previous}')
        config['resume_from'] = str(previous)
    settings.validate_config(config, calibration_validation=args.calibration_validation)
    tcp_path = settings.resolve(config.get('tcp_calibration'))
    guard = None
    if tcp_path and tcp_path.exists():
        guard = settings.load_tcp_calibration(tcp_path)
        settings.check_target_roll_goals(config, guard)
    elif not args.calibration_validation:
        raise ValueError(f'TCP calibration {tcp_path} not found: only --calibration-validation '
                         '(exactly 1 cycle, marker-8 values only) may run without it')
    return config, guard


# ======================================================================== session
class Benchmark:
    """Visit sequencing with injected robot/cameras/recorder (mockable in tests)."""

    def __init__(self, config, *, robot, external, onboard, rectifier, detector, recorder,
                 k, distortion, guard, fingerprint_paths, fingerprint, controls_check=None,
                 stop_event=None, log=print, metrology=None):
        self.config = config
        self.robot = robot
        self.external = external
        self.onboard = onboard
        self.rectifier = rectifier
        self.detector = detector
        self.recorder = recorder
        self.k = k
        self.d = distortion
        self.guard = guard
        self.paths = fingerprint_paths
        self.expected = fingerprint
        self.fingerprint_sha = settings.canonical_hash(fingerprint)
        self.config_sha = settings.canonical_hash(config)
        self.controls_check = controls_check
        self.stop_event = stop_event or threading.Event()
        self.log = log
        self.metrology = metrology
        # guard: TCP calibration (matrix + joint guard) or, in calibration-validation
        # mode only, a provisional joint guard from the start pose with matrix None.
        self.marker_tcp = guard.get('matrix') if guard else None
        self.camera_failure_visits = 0
        self.cycle_seconds = []

    # ------------------------------------------------------------ guards
    def _check_stop(self):
        if self.stop_event.is_set():
            raise E.Interrupted('stop requested')

    def _guard_now(self, where):
        def waiting(details):
            self.recorder.event('joint_feedback_wait_started', dict(details, where=where))
            self.log(f'{where}: waiting up to {details["timeout_s"]:g} s for fresh joint feedback')

        joints, age, recovery = settings.wait_for_joint_feedback(
            self.robot.joints, self.config, self.stop_event, where, on_wait=waiting)
        if recovery is not None:
            self.recorder.event('joint_feedback_recovered', dict(recovery, where=where))
            self.log(f'{where}: joint feedback recovered after {recovery["wait_s"]:.2f} s')
        return joints, age, settings.check_joint_guard(joints, self.guard, where)

    def preflight_visit(self):
        settings.check_fingerprint(self.paths, self.expected)
        if self.controls_check:
            try:
                self.controls_check()
            except ValueError as error:
                raise E.SafetyRejection(str(error)) from error
        if not self.external.healthy():
            raise E.CameraFailure('external camera capture thread failed')

    # ------------------------------------------------------------ measurement
    def _pose_frame(self, image, target):
        """Detect and estimate. Returns (row fields, valid, transforms)."""
        gripper = self.config['gripper_id']
        try:
            found = self.detector.detect(image)
        except ValueError as error:
            return {'failure_category': E.INVALID_POSE_ESTIMATE, 'notes': str(error), 'corners': {}}, None
        row = {'corners': {str(i): c.tolist() for i, c in found.items()}}
        if target not in found:
            row['failure_category'] = E.ARUCO_TARGET_NOT_DETECTED
            if gripper not in found:
                row['notes'] = E.ARUCO_GRIPPER_NOT_DETECTED
            return row, None
        if gripper not in found:
            row['failure_category'] = E.ARUCO_GRIPPER_NOT_DETECTED
            return row, None
        try:
            camera_target, target_error = self.detector.pose(target, found[target], self.k, self.d)
            camera_marker, marker_error = self.detector.pose(gripper, found[gripper], self.k, self.d)
        except ValueError as error:
            row.update(failure_category=E.INVALID_POSE_ESTIMATE, notes=str(error))
            return row, None
        target_marker = geometry.target_marker(camera_target, camera_marker)
        row.update(T_camera_target=camera_target.tolist(), T_camera_marker=camera_marker.tolist(),
                   T_target_marker=target_marker.tolist(), target_reprojection_px=target_error,
                   marker_reprojection_px=marker_error)
        target_tcp = None
        if self.marker_tcp is not None:
            target_tcp = geometry.target_tcp(target_marker, self.marker_tcp)
            row['T_target_tcp'] = target_tcp.tolist()
        return row, (camera_target, camera_marker, target_marker, target_tcp, found[gripper])

    def _ball_crosscheck(self, image, poses):
        camera_target, camera_marker, _, target_tcp, marker_corners = poses
        try:
            if self.metrology is None:
                import metrology
                self.metrology = metrology
            center, quality, _, _ = self.metrology.orange_sphere(
                image, self.k, self.d, camera_marker, marker_corners,
                diameter_m=self.config['ball']['diameter_m'])
        except ImportError as error:
            return {'ball_status': f'unavailable: {error}'}
        except Exception as error:  # segmentation/fit failures are logged, never hidden
            return {'ball_status': 'failed', 'ball_notes': str(error)[:500]}
        ball = geometry.point_in_target(camera_target, center)
        result = {'ball_status': 'ok', 'ball_center_target_mm': (ball * 1000).tolist(), 'ball_quality': quality}
        if target_tcp is not None:
            result['ball_error_mm'] = float(np.linalg.norm(ball - target_tcp[:3, 3]) * 1000)
        return result

    def _onboard_pair(self, visit, external_frame_id, external_time, state):
        robot_camera = self.config['robot_camera']
        row = {'frame_id': f'{visit["visit_id"]}_onboard_{state["pairs"]:03d}', 'visit_id': visit['visit_id'],
               'cycle': visit['cycle'], 'target_id': visit['target_id'], 'camera': 'onboard',
               'paired_external_frame': external_frame_id, 'valid': False}
        state['pairs'] += 1
        try:
            seq, mono, stamp, raw = self.onboard.frame(after=state['onboard_seq'], timeout=robot_camera['pair_timeout_s'],
                                                       max_age_s=robot_camera['max_frame_age_s'])
        except E.CameraFailure as error:
            row.update(failure_category=E.CAMERA_CAPTURE_FAILURE, notes=str(error))
            state['issues']['missing_or_stale'] += 1
            self.recorder.frame(row, {})
            return
        state['onboard_seq'] = seq
        skew = mono - external_time
        row.update(sequence=seq, acquired_monotonic=mono, acquired_utc=stamp, skew_s=skew,
                   frame_age_s=time.monotonic() - mono)
        state['skews'].append(abs(skew))
        if abs(skew) > robot_camera['max_pair_skew_s']:
            row['notes'] = f'pair skew {skew:.3f} s exceeds {robot_camera["max_pair_skew_s"]} s'
            state['issues']['skew'] += 1
        images = {'raw_stereo': (raw, 'png')}
        try:
            left = self.rectifier.left(raw)
            images['rectified_left'] = (left, self.config['storage']['onboard_rectified_format'])
            found = self.detector.detect(left)
            row['corners'] = {str(i): c.tolist() for i, c in found.items()}
            row['robot_target_visible'] = visit['target_id'] in found
            row['valid'] = True
            state['visible'] = state['visible'] or row['robot_target_visible']
        except (ValueError, E.CameraFailure) as error:
            row.update(failure_category=E.INVALID_POSE_ESTIMATE, notes=str(error))
            state['issues']['analysis'] += 1
        self.recorder.frame(row, images)

    def measure(self, visit):
        config = self.config
        required = config['valid_frames']
        minimum = config.get('min_valid_frames', required)
        target = visit['target_id']
        cutoff = time.monotonic()
        end = cutoff + config['capture_timeout_s']
        valid = []
        failures = collections.Counter()
        digests = set()
        sequence = 0
        analyzed = 0
        crosscheck = {'attempts': 0, 'ok': 0, 'errors_mm': []}
        onboard = {'pairs': 0, 'onboard_seq': 0, 'skews': [], 'issues': collections.Counter(), 'visible': False}
        pair_every = (config.get('robot_camera') or {}).get('pair_every_n_external_frames', 0)
        ball_every = config['ball'].get('crosscheck_every_n_valid_frames', 0)
        while time.monotonic() < end and len(valid) < required:
            self._check_stop()
            frame_id = f'{visit["visit_id"]}_external_{analyzed:03d}'
            base = {'frame_id': frame_id, 'visit_id': visit['visit_id'], 'cycle': visit['cycle'],
                    'target_id': target, 'camera': 'external', 'analysis_index': analyzed, 'valid': False}
            try:
                sequence, mono, stamp, image = self.external.frame(
                    after=sequence, timeout=max(0.05, min(2.0, end - time.monotonic())))
            except E.CameraFailure as error:
                failures[E.CAMERA_CAPTURE_FAILURE] += 1
                analyzed += 1
                self.recorder.frame(dict(base, failure_category=E.CAMERA_CAPTURE_FAILURE, notes=str(error)), {})
                if not self.external.healthy():
                    break
                continue
            if mono < cutoff:
                continue  # acquired before settling ended: not an analysis attempt
            analyzed += 1
            joints, age, _ = self._guard_now(f'{visit["visit_id"]} frame {analyzed}')
            row = dict(base, sequence=sequence, acquired_monotonic=mono, acquired_utc=stamp,
                       joints=joints, joint_age_s=age, wrist_roll_rad=joints.get(ROLL_JOINT),
                       gripper_m=joints.get(GRIPPER_JOINT))
            digest = hashlib.blake2b(image.tobytes(), digest_size=16).hexdigest()
            row['pixel_digest'] = digest
            if digest in digests:
                row['failure_category'] = E.DUPLICATE_FRAME
                poses = None
            else:
                digests.add(digest)
                fields, poses = self._pose_frame(image, target)
                row.update(fields)
            if poses is not None:
                row['valid'] = True
                row['failure_category'] = ''
                valid.append((poses[2], poses[3], stamp))
                if ball_every and (len(valid) - 1) % ball_every == 0:
                    crosscheck['attempts'] += 1
                    row.update(self._ball_crosscheck(image, poses))
                    if row.get('ball_status') == 'ok':
                        crosscheck['ok'] += 1
                        if 'ball_error_mm' in row:
                            crosscheck['errors_mm'].append(row['ball_error_mm'])
            else:
                failures[row['failure_category']] += 1
            self.recorder.frame(row, {'external': (image, self.config['storage']['image_format'])})
            if self.onboard is not None and pair_every and (analyzed - 1) % pair_every == 0:
                self._onboard_pair(visit, frame_id, mono, onboard)
        self.recorder.barrier()  # every analyzed frame durable before the measurement row
        result = {'valid_frames': len(valid), 'analyzed_frames': analyzed, 'required_frames': minimum,
                  'requested_frames': required, 'frame_target_reached': len(valid) >= required,
                  'frame_failures': dict(failures), 'detection_success': len(valid) >= minimum,
                  'measurement_frame': 'target_to_tcp_and_marker8' if self.marker_tcp is not None
                  else 'target_to_marker8_only',
                  'onboard_frames': onboard['pairs'],
                  'onboard_max_abs_skew_s': max(onboard['skews']) if onboard['skews'] else None,
                  'onboard_issues': dict(onboard['issues']),
                  'robot_target_visible': onboard['visible'] if self.onboard is not None else None,
                  'ball_crosscheck_attempts': crosscheck['attempts'], 'ball_crosscheck_ok': crosscheck['ok'],
                  'ball_crosscheck_median_mm': float(np.median(crosscheck['errors_mm'])) if crosscheck['errors_mm'] else None}
        if valid:
            marker = geometry.median_xyz_mm([v[0] for v in valid]).tolist()
            tcp = geometry.median_xyz_mm([v[1] for v in valid]).tolist() if self.marker_tcp is not None else None
            result['timestamp'] = valid[len(valid) // 2][2]
            if result['detection_success']:
                result.update(marker_xyz_mm=marker, tcp_xyz_mm=tcp)
            else:  # partial medians kept for diagnosis only; never a valid result
                result.update(partial_marker_xyz_mm=marker, partial_tcp_xyz_mm=tcp)
        if not result['detection_success']:
            result['failure_category'] = (failures.most_common(1)[0][0] if failures else E.CAMERA_CAPTURE_FAILURE)
        return result

    # ------------------------------------------------------------ visit
    def visit(self, cycle, target):
        config = self.config
        started = time.monotonic()
        visit = {'visit_id': f'c{cycle:03d}_t{target}', 'cycle': cycle, 'target_id': target, 'repetition': cycle,
                 'status': 'started', 'started_utc': utc(), 'motion_success': False, 'home_success': False,
                 'measurement_valid': False, 'settling_s': config['settling_s'],
                 'config_sha256': self.config_sha, 'fingerprint_sha256': self.fingerprint_sha,
                 'commanded_joints': config['targets'][str(target)]['joints'],
                 'tcp_calibration_sha256': self.guard['sha256'] if self.guard else None}
        joints = config['targets'][str(target)]['joints']
        fatal = None
        acked = False
        try:
            self.preflight_visit()
            pre_joints, _, guard_errors = self._guard_now(f'{visit["visit_id"]} before motion')
            visit.update(joints_before=pre_joints, guard_errors_before=guard_errors)
            self.recorder.start_visit(visit)  # durable ACK before any motion
            acked = True
            motion_start = time.monotonic()
            try:
                visit['motion'] = self.robot.go(joints, label=f'{visit["visit_id"]} outbound')
            except E.MotionFailure as error:
                visit['motion'] = getattr(error, 'log', None)
                raise
            visit.update(motion_success=True, motion_s=time.monotonic() - motion_start, status='at_target',
                         max_joint_error_rad=visit['motion']['max_abs_error_rad'],
                         transit_roll_excursion_rad=visit['motion'].get('transit_roll_excursion_rad'))
            self.recorder.update_visit(visit)
            self._guard_now(f'{visit["visit_id"]} at target')
            if self.stop_event.wait(config['settling_s']):
                raise E.Interrupted('stop requested during settling')
            capture_start = time.monotonic()
            visit.update(self.measure(visit))
            at_joints, _, guard_errors = self._guard_now(f'{visit["visit_id"]} after measurement')
            visit.update(capture_s=time.monotonic() - capture_start, joints_at_measurement=at_joints,
                         guard_errors_at_measurement=guard_errors, wrist_roll_rad=at_joints.get(ROLL_JOINT),
                         gripper_m=at_joints.get(GRIPPER_JOINT),
                         measurement_valid=bool(visit['detection_success']), status='measured')
            self.recorder.update_visit(visit)  # durable measurement BEFORE returning home
            if visit.get('failure_category') == E.CAMERA_CAPTURE_FAILURE:
                self.camera_failure_visits += 1
            else:
                self.camera_failure_visits = 0
            home_start = time.monotonic()
            try:
                visit['home_motion'] = self.robot.go(self.robot.home, label=f'{visit["visit_id"]} home')
            except E.MotionFailure as error:
                visit['home_motion'] = getattr(error, 'log', None)
                raise
            visit.update(home_success=True, home_s=time.monotonic() - home_start)
            visit['status'] = 'complete' if visit['measurement_valid'] else 'failed'
        except (E.BenchmarkError, KeyboardInterrupt, TimeoutError) as error:
            fatal = error
            visit.update(status='failed', failure_category=E.category_of(error),
                         notes=(visit.get('notes') or '') + repr(error))
        visit.update(visit_s=time.monotonic() - started, finished_utc=utc())
        if isinstance(fatal, E.PersistenceFailure):
            raise fatal  # nothing more can be recorded reliably
        if acked:
            self.recorder.update_visit(visit)
        else:
            self.recorder.event('visit_rejected_before_motion', {'visit': visit}, durable=True)
        self.log(f'cycle={cycle} target={target} status={visit["status"]} valid={visit.get("valid_frames", 0)}/'
                 f'{config["valid_frames"]} motion={visit["motion_success"]} home={visit["home_success"]} '
                 f'fail={visit.get("failure_category", "") or "-"} {visit["visit_s"]:.1f}s')
        if fatal is not None:
            raise fatal
        if self.camera_failure_visits >= config['max_consecutive_camera_failure_visits']:
            raise E.CameraFailure(f'{self.camera_failure_visits} consecutive visits without camera frames')
        return visit

    def run(self):
        """Run all cycles. Raises on motion/safety/persistence/interrupt (robot left in place)."""
        total = self.config['cycles']
        first_cycle = self.config.get('start_cycle', 1)
        for cycle in range(first_cycle, total + 1):
            self._check_stop()
            start = time.monotonic()
            targets = self.config['target_ids']
            if cycle == first_cycle and 'start_target' in self.config:
                targets = targets[targets.index(self.config['start_target']):]
            visits = [self.visit(cycle, target) for target in targets]
            seconds = time.monotonic() - start
            self.cycle_seconds.append(seconds)
            completed = sum(v['status'] == 'complete' for v in visits)
            self.recorder.cycle(cycle, seconds, completed, {'writer': dict(self.recorder.stats)})
            mean = float(np.mean(self.cycle_seconds))
            self.log(f'cycle {cycle}/{total}: {seconds:.1f} s, {completed}/{len(visits)} complete; '
                     f'mean {mean:.1f} s/cycle; remaining ~{mean * (total - cycle) / 60:.1f} min; '
                     f'{total} cycles ~{mean * total / 60:.1f} min')


# ======================================================================== runtime
class StopSignals:
    """SIGINT/SIGTERM set an event; ROS context stays alive to cancel goals."""

    def __init__(self, log=print):
        self.event = threading.Event()
        self.log = log
        self.previous = {}

    def __enter__(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.previous[sig] = signal.signal(sig, self._handle)
        return self.event

    def _handle(self, signum, _frame):
        if self.event.is_set():
            self.log('stop already requested; waiting for goal cancellation and recorder drain', file=sys.stderr)
        else:
            self.log(f'{signal.Signals(signum).name}: cancelling active motion; robot will be left in place',
                     file=sys.stderr)
        self.event.set()

    def __exit__(self, *exc):
        for sig, handler in self.previous.items():
            signal.signal(sig, handler)


def open_cameras(config):
    import vision
    ext_cfg = config['external_camera']
    calibration_path = settings.resolve(ext_cfg['calibration'])
    k, d, calibration = vision.load_intrinsics(calibration_path, ext_cfg['resolution'], ext_cfg['device'])
    controls = vision.read_controls(ext_cfg['device'])
    vision.check_controls(controls, calibration)
    external = onboard = rectifier = None
    try:
        external = vision.Camera(ext_cfg['device'], ext_cfg['resolution'], fourcc=ext_cfg['fourcc'],
                                 fps=ext_cfg['fps'], name='external')
        robot_cfg = config.get('robot_camera') or {}
        if robot_cfg.get('device'):
            width, height = robot_cfg['resolution']
            rectifier = vision.StereoRectifier(settings.resolve(robot_cfg['stereo_calibration']), (width // 2, height))
            onboard = vision.Camera(robot_cfg['device'], robot_cfg['resolution'], fourcc=robot_cfg['fourcc'],
                                    fps=robot_cfg['fps'], name='onboard')
        external.frame(timeout=5)
        if onboard:
            onboard.frame(timeout=5)
    except BaseException:
        for camera in (external, onboard):
            if camera:
                camera.close()
        raise
    def controls_check():
        vision.check_controls(vision.read_controls(ext_cfg['device']), calibration)
    return external, onboard, rectifier, k, d, controls, controls_check


def provisional_guard(config, joints):
    """Calibration-validation only: hold roll/gripper at their start values (no TCP matrix)."""
    tolerances = config['provisional_guard_tolerances']
    return {'path': None, 'sha256': None, 'matrix': None, 'provisional': True,
            'required': {j: float(joints[j]) for j in settings.GUARDED_JOINTS},
            'tolerances': {j: float(tolerances[j]) for j in settings.GUARDED_JOINTS}}


def write_session(session, **state):
    import storage
    path = session / 'session.json'
    current = json.loads(path.read_text()) if path.exists() else {}
    current.update(state, updated_utc=utc())
    storage.atomic_json(path, current)


def check_disk(config, session_parent):
    import shutil
    session_parent.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(session_parent).free
    estimate = settings.estimate_storage_bytes(config)
    minimum = config['storage']['min_free_gb'] * 1e9
    return {'free_bytes': free, 'estimated_bytes_upper': estimate, 'min_free_bytes': minimum,
            'ok': free > estimate + minimum}


def preflight(config, guard, args):
    """Offline: no ROS, no cameras, no motion. --live adds read-only hardware checks + plans."""
    report = {'utc': utc(), 'cycles': config['cycles'], 'velocity_scaling': config['velocity_scaling'],
              'acceleration_scaling': config['acceleration_scaling'], 'target_order': config['target_ids'],
              'tcp_calibration': guard['path'] if guard else None,
              'mode': 'calibration-validation (marker-8 only)' if guard is None else 'TCP + marker-8',
              'disk': check_disk(config, (args.output.parent if args.output else settings.SESSIONS)),
              'visits': config['cycles'] * len(config['target_ids'])}
    if guard:
        report['guard'] = {'required': guard['required'], 'tolerances': guard['tolerances']}
    if not args.live:
        report['note'] = 'Offline preflight: nothing was opened, planned or executed.'
        print(json.dumps(report, indent=2))
        return 0 if report['disk']['ok'] else 2
    from robot import Robot
    stop = threading.Event()
    robot = Robot(config, stop)
    cameras = None
    try:
        robot.wait_ready()
        report['controllers'] = robot.controller_states()
        joints, age = robot.joints()
        report['joints'] = joints
        report['guard_errors'] = settings.check_joint_guard(joints, guard, 'preflight') if guard else None
        cameras = open_cameras(config)
        external, onboard, rectifier, k, d, controls, _ = cameras
        report['external_controls'] = controls
        import vision
        detector = vision.Markers(config)
        report['external_visible_ids'] = sorted(detector.detect(external.frame()[3]))
        if onboard:
            report['onboard_visible_ids_rectified_left'] = sorted(detector.detect(rectifier.left(onboard.frame()[3])))
        robot.add_obstacles()
        plans = {}
        for target in config['target_ids']:
            pose = config['targets'][str(target)]['joints']
            plans[str(target)] = {'outbound': robot.plan(pose, start=robot.home)[0],
                                  'homebound': robot.plan(robot.home, start=pose)[0]}
        plans['initial_to_reference'] = robot.plan(robot.home)[0]
        report['plans'] = plans
        report['note'] = 'Live preflight: read-only checks and plan-only requests; no motion was executed.'
        if report['controllers'].get('arm_controller') != 'active':
            report['problem'] = 'arm_controller is not active (run.sh activate-arm)'
        print(json.dumps(report, indent=2, default=str))
        return 0 if report['disk']['ok'] and 'problem' not in report else 2
    finally:
        if cameras:
            for camera in cameras[:2]:
                if camera:
                    camera.close()
        robot.close()


def run(config, guard, args, config_path):
    import storage
    import vision
    disk = check_disk(config, (args.output.parent if args.output else settings.SESSIONS))
    if not disk['ok']:
        raise E.PersistenceFailure(f'not enough free disk space: {disk}')
    mode = 'validation' if args.calibration_validation else 'run'
    session = settings.new_session_dir(mode, output=args.output)
    print(f'Session: {session}', flush=True)
    paths = settings.fingerprint_paths(config, config_path)
    expected = settings.fingerprint(paths)
    write_session(session, status='starting', mode=mode, cycles=config['cycles'], pid=os.getpid())
    manifest = settings.snapshot(session, config, config_path, paths,
                                 {'fingerprint': expected, 'disk': disk, 'argv': sys.argv,
                                  'tcp_guard': {k: guard[k] for k in ('path', 'sha256', 'required', 'tolerances')}
                                  if guard else None})
    store = config['storage']
    recorder = storage.Recorder(session, queue_size=store['queue_size'], image_format=store['image_format'],
                                png_compression=store['png_compression'], jpeg_quality=store['jpeg_quality'],
                                min_free_bytes=store['min_free_gb'] * 1e9,
                                max_submit_block_s=store['max_submit_block_s'])
    recorder.meta('manifest', manifest)
    robot = cameras = None
    status = 'failed_before_motion'
    exit_code = 1
    with StopSignals() as stop:
        try:
            cameras = open_cameras(config)
            external, onboard, rectifier, k, d, controls, controls_check = cameras
            recorder.event('cameras_opened', {'external_controls': controls,
                                              'external': config['external_camera'],
                                              'robot': config.get('robot_camera')}, durable=True)
            from robot import Robot
            robot = Robot(config, stop)
            robot.wait_ready()
            controllers = robot.controller_states()
            if controllers.get('arm_controller') != 'active':
                raise E.SafetyRejection(f'arm_controller not active: {controllers}')
            joints, age = robot.joints()
            if guard is None:
                guard = provisional_guard(config, joints)
                settings.check_target_roll_goals(config, guard)
                recorder.event('provisional_joint_guard', {k: guard[k] for k in ('required', 'tolerances')},
                               durable=True)
            settings.check_joint_guard(joints, guard, 'start')
            robot.add_obstacles()
            plans = {}
            for target in config['target_ids']:
                pose = config['targets'][str(target)]['joints']
                plans[str(target)] = {'outbound': robot.plan(pose, start=robot.home)[0],
                                      'homebound': robot.plan(robot.home, start=pose)[0]}
            recorder.event('preflight', {'controllers': controllers, 'joints': joints, 'joint_age_s': age,
                                         'plans': plans, 'reference': robot.home}, durable=True)
            benchmark = Benchmark(config, robot=robot, external=external, onboard=onboard, rectifier=rectifier,
                                  detector=vision.Markers(config), recorder=recorder, k=k, distortion=d,
                                  guard=guard, fingerprint_paths=paths, fingerprint=expected,
                                  controls_check=controls_check, stop_event=stop)
            benchmark.preflight_visit()
            recorder.event('initial_reference_motion_start', {}, durable=True)
            write_session(session, status='running')
            status = 'running'
            log = robot.go(robot.home, label='initial reference')
            recorder.event('initial_reference_motion', log, durable=True)
            benchmark.run()
            final, _ = robot.joints()
            recorder.event('finished', {'final_joints': final, 'cycle_seconds': benchmark.cycle_seconds}, durable=True)
            status = 'finished'
            exit_code = 0
            print('Finished: arm at reference pose, ball still held (no release commanded).', flush=True)
        except (Exception, KeyboardInterrupt) as error:
            category = E.category_of(error)
            status = f'stopped_{category.lower()}' if status == 'running' else f'{status}_{category.lower()}'
            print(f'STOP [{category}]: {error}', file=sys.stderr, flush=True)
            print('Robot left in its current pose; no automatic homing after a fault.', file=sys.stderr, flush=True)
            cancel = getattr(robot, 'last_cancel', None) if robot else None
            try:
                recorder.event('stopped', {'category': category, 'error': repr(error), 'cancel': cancel}, durable=True)
            except E.PersistenceFailure as persistence:
                print(f'recorder unavailable: {persistence}', file=sys.stderr)
        finally:
            for camera in (cameras or (None, None))[:2]:
                if camera:
                    camera.close()
            try:
                recorder.close()
            except E.PersistenceFailure as error:
                status = 'persistence_failure'
                exit_code = 1
                print(f'PERSISTENCE FAILURE while closing: {error}; run recover', file=sys.stderr)
            if robot:
                robot.close()
            try:
                write_session(session, status=status, writer=recorder.stats)
            except OSError as error:
                print(f'cannot update session.json: {error}', file=sys.stderr)
    print(f'Session data: {session}\nReport: ./benchmarks/physical_repeatability/run.sh report {session}', flush=True)
    return exit_code


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config, guard = prepare_config(args)
    except (ValueError, E.SafetyRejection, OSError) as error:
        parser.error(str(error))
    if args.command == 'preflight':
        return preflight(config, guard, args)
    if args.live:
        parser.error('--live is only valid with preflight')
    return run(config, guard, args, args.config.resolve())


if __name__ == '__main__':
    raise SystemExit(main())
