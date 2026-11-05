"""Configuration, TCP calibration and fingerprint guards (no ROS, no camera access)."""
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import secrets
import shutil
import subprocess
import sys
import time

import numpy as np

import geometry
from errors import Interrupted, SafetyRejection

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_CONFIG = HERE / 'config.json'
SESSIONS = HERE / 'sessions'

ARM_JOINTS = ['limb1_to_base_link_joint', 'limb2_to_limb1_joint', 'limb3_to_limb2_joint',
              'limb4_to_limb3_joint', 'limb5_to_limb4_joint']
ROLL_JOINT = 'limb5_to_limb4_joint'
GRIPPER_JOINT = 'gripper_left_joint'
GUARDED_JOINTS = [ROLL_JOINT, GRIPPER_JOINT]

# Robot model / planning settings fingerprinted at start and before every visit.
ROBOT_MODEL_FILES = [
    'app/src/hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.srdf',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/joint_limits.yaml',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/kinematics.yaml',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/moveit_controllers.yaml',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/ros2_controllers.yaml',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.urdf.xacro',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.ros2_control.xacro',
    'app/src/hiwonder_xarm_esp32_moveit_config/config/initial_positions.yaml',
    'app/src/hiwonder_xarm_esp32_moveit_config/launch/hardware.launch.py',
]
ROBOT_MODEL_GLOBS = ['app/src/hiwonder_xarm_esp32_description/urdf/*']
SOFTWARE_GLOBS = ['*.py', '*.sh', 'launch/*', 'tests/*.py', 'config.json']


def utc():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def resolve(path):
    if path in (None, '', 'none'):
        return None
    path = Path(os.path.expanduser(str(path)))
    return path if path.is_absolute() else (HERE / path).resolve()


def load_config(path=DEFAULT_CONFIG):
    return json.loads(Path(path).read_text())


def apply_overrides(config, args):
    config = copy.deepcopy(config)
    for key in ('cycles', 'velocity_scaling', 'acceleration_scaling'):
        value = getattr(args, key, None)
        if value is not None:
            config[key] = value
    if getattr(args, 'external_camera', None):
        config['external_camera']['device'] = args.external_camera
    if getattr(args, 'robot_camera', None):
        config['robot_camera']['device'] = None if args.robot_camera == 'none' else args.robot_camera
    if getattr(args, 'tcp_calibration', None):
        config['tcp_calibration'] = args.tcp_calibration
    return config


def validate_config(config, *, calibration_validation=False):
    """Static checks. Raises ValueError with a human-readable reason."""
    errors = []
    cycles = config.get('cycles')
    if not isinstance(cycles, int) or cycles < 1:
        errors.append('cycles must be an integer >= 1')
    first = config.get('start_cycle', 1)
    if not isinstance(first, int) or not isinstance(cycles, int) or not 1 <= first <= cycles:
        errors.append('start_cycle must be an integer between 1 and cycles')
    if not 0 < float(config.get('velocity_scaling', 0)) <= 1:
        errors.append('velocity_scaling must be in (0, 1]')
    if not 0 < float(config.get('acceleration_scaling', 0)) <= 1:
        errors.append('acceleration_scaling must be in (0, 1]')
    ids = config.get('target_ids', [])
    if ids != sorted(set(ids)) or not ids:
        errors.append('target_ids must be unique and strictly increasing')
    if 'start_target' in config and config['start_target'] not in ids:
        errors.append('start_target must belong to target_ids')
    gripper = config.get('gripper_id')
    if gripper in ids:
        errors.append('gripper marker id must not be a fixed target id')
    sizes = config.get('marker_sizes_mm', {})
    for marker in list(ids) + [gripper]:
        if not float(sizes.get(str(marker), 0)) > 0:
            errors.append(f'missing positive marker size for id {marker}')
    if config.get('valid_frames', 0) < 2:
        errors.append('valid_frames must be >= 2')
    minimum = config.get('min_valid_frames', config.get('valid_frames', 0))
    if not isinstance(minimum, int) or not 2 <= minimum <= config.get('valid_frames', 0):
        errors.append('min_valid_frames must be an integer between 2 and valid_frames')
    if config.get('settling_s', -1) < 0 or config.get('capture_timeout_s', 0) <= 0:
        errors.append('settling_s must be >= 0 and capture_timeout_s > 0')
    recovery_timeout = float(config.get('joint_state_recovery_timeout_s', 60.0))
    if not np.isfinite(recovery_timeout) or recovery_timeout < 0:
        errors.append('joint_state_recovery_timeout_s must be finite and >= 0')
    if config.get('orientation_enabled'):
        errors.append('orientation reporting is not supported: TCP orientation is not calibrated')
    if not config.get('workspace_validated') or not config.get('obstacles'):
        errors.append('workspace collision geometry must be validated and recorded')
    for marker in ids:
        pose = config.get('targets', {}).get(str(marker), {})
        joints = pose.get('joints') or {}
        if set(joints) != set(ARM_JOINTS) or not all(np.isfinite(list(joints.values()))):
            errors.append(f'target {marker} has no complete frozen arm joint pose')
        elif not pose.get('validated'):
            errors.append(f'target {marker} pose is not physically validated')
    if calibration_validation and cycles != 1:
        errors.append('--calibration-validation is only allowed with exactly 1 cycle')
    external = config.get('external_camera') or {}
    if not external.get('device') or not external.get('calibration'):
        errors.append('external camera device and calibration are required')
    storage = config.get('storage', {})
    if storage.get('queue_size', 0) < 1:
        errors.append('storage.queue_size must be >= 1')
    if errors:
        raise ValueError('; '.join(errors))


def load_tcp_calibration(path):
    """Validate root-provided marker8 -> operational TCP translation.

    Accepted keys: "matrix" or "T_marker_tcp" (4x4, metres, identity rotation),
    "required_joint_positions" and "tolerances" (or "joint_tolerances") for both
    the wrist roll and gripper joints, and a non-empty "provenance".
    """
    path = Path(path)
    data = json.loads(path.read_text())
    matrix = np.asarray(data.get('T_marker_tcp', data.get('matrix')), float)
    if not geometry.is_rigid(matrix):
        raise ValueError(f'{path}: T_marker_tcp/matrix must be a proper 4x4 rigid transform')
    if not np.allclose(matrix[:3, :3], np.eye(3), atol=1e-9):
        raise ValueError(f'{path}: rotation must be identity; TCP orientation is not calibrated')
    if np.linalg.norm(matrix[:3, 3]) > 0.5:
        raise ValueError(f'{path}: translation must be in metres (got > 0.5 m)')
    required = data.get('required_joint_positions') or {}
    tolerances = data.get('tolerances', data.get('joint_tolerances')) or {}
    for joint in GUARDED_JOINTS:
        if joint not in required or not np.isfinite(required[joint]):
            raise ValueError(f'{path}: required_joint_positions.{joint} missing')
        if not float(tolerances.get(joint, 0)) > 0:
            raise ValueError(f'{path}: positive tolerance for {joint} missing')
    if not data.get('provenance'):
        raise ValueError(f'{path}: provenance missing')
    if int(data.get('marker_id', 8)) != 8:
        raise ValueError(f'{path}: marker_id must be 8')
    return {'path': str(path), 'sha256': sha256_file(path), 'matrix': matrix,
            'required': {j: float(required[j]) for j in GUARDED_JOINTS},
            'tolerances': {j: float(tolerances[j]) for j in GUARDED_JOINTS},
            'raw': data}


def check_target_roll_goals(config, guard):
    """Planned (commanded) wrist-roll goals must equal the calibration roll."""
    required = guard['required'][ROLL_JOINT]
    tolerance = guard['tolerances'][ROLL_JOINT]
    for marker in config['target_ids']:
        roll = config['targets'][str(marker)]['joints'][ROLL_JOINT]
        if abs(roll - required) > tolerance:
            raise SafetyRejection(f'target {marker} wrist roll goal {roll:.6f} rad differs from TCP '
                                  f'calibration {required:.6f} +/- {tolerance} rad')


def wait_for_joint_feedback(read_joints, config, stop_event, where='', on_wait=None):
    """Wait without commanding motion until feedback is fresh, or stop on timeout.

    Freshness remains strict: the recovery timeout does not make an old sample
    acceptable. Return recovery timing so callers can preserve the interruption.
    """
    started = time.monotonic()
    timeout = float(config.get('joint_state_recovery_timeout_s', 60.0))
    recovery = None
    while True:
        if stop_event.is_set():
            raise Interrupted(f'{where}: stop requested while waiting for joint feedback')
        joints, age = read_joints()
        if np.isfinite(age) and 0 <= age <= config['joint_state_max_age_s']:
            if recovery is not None:
                recovery['wait_s'] = time.monotonic() - started
            return joints, age, recovery
        if recovery is None:
            recovery = {'initial_age_s': float(age) if np.isfinite(age) else None,
                        'timeout_s': timeout}
            if on_wait is not None:
                on_wait(dict(recovery))
        remaining = timeout - (time.monotonic() - started)
        if remaining <= 0:
            raise SafetyRejection(f'{where}: joint feedback stale ({age:.2f} s) '
                                  f'after waiting {timeout:g} s for fresh feedback')
        stop_event.wait(min(0.02, remaining))


def check_joint_guard(joints, guard, where=''):
    """Measured wrist roll and gripper opening must stay at calibration values."""
    if guard is None:
        return {}
    errors = {}
    for joint in GUARDED_JOINTS:
        if joint not in joints:
            raise SafetyRejection(f'{where}: no feedback for {joint}')
        errors[joint] = float(joints[joint]) - guard['required'][joint]
        if abs(errors[joint]) > guard['tolerances'][joint]:
            raise SafetyRejection(f'{where}: {joint}={joints[joint]:.6f} outside TCP calibration '
                                  f'{guard["required"][joint]:.6f} +/- {guard["tolerances"][joint]}')
    return errors


def fingerprint_paths(config, config_path):
    paths = {'config': Path(config_path)}
    external = resolve(config['external_camera'].get('calibration'))
    if external:
        paths['external_calibration'] = external
    tcp = resolve(config.get('tcp_calibration'))
    if tcp and tcp.exists():
        paths['tcp_calibration'] = tcp
    stereo = resolve((config.get('robot_camera') or {}).get('stereo_calibration'))
    if stereo:
        paths['stereo_calibration'] = stereo
    for relative in ROBOT_MODEL_FILES:
        path = ROOT / relative
        if path.is_file():
            paths['robot_model:' + relative] = path
    for pattern in ROBOT_MODEL_GLOBS:
        for path in sorted(ROOT.glob(pattern)):
            if path.is_file():
                paths['robot_model:' + str(path.relative_to(ROOT))] = path
    return paths


def fingerprint(paths):
    result = {}
    for name, path in paths.items():
        result[name] = sha256_file(path) if Path(path).is_file() else 'MISSING'
    return result


def check_fingerprint(paths, expected):
    current = fingerprint(paths)
    changed = sorted(k for k in set(current) | set(expected) if current.get(k) != expected.get(k))
    if changed:
        raise SafetyRejection('configuration/calibration files changed during the session: ' + ', '.join(changed))
    return current


def new_session_dir(mode, base=SESSIONS, output=None):
    """Fresh unique directory; never reuses or overwrites an existing one."""
    if output is not None:
        directory = Path(output)
    else:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        directory = Path(base) / f'{stamp}_{mode}_{secrets.token_hex(3)}'
    directory.parent.mkdir(parents=True, exist_ok=True)
    directory.mkdir(exist_ok=False)
    return directory


def estimate_storage_bytes(config):
    """Conservative upper estimate used by preflight (PNG ~<= raw size)."""
    ext_w, ext_h = config['external_camera']['resolution']
    frames = config['valid_frames'] + 10
    per_visit = frames * ext_w * ext_h * 3 * 1.05
    robot = config.get('robot_camera') or {}
    if robot.get('device'):
        raw_w, raw_h = robot['resolution']
        pairs = frames // max(1, robot.get('pair_every_n_external_frames', 5)) + 1
        per_visit += pairs * (raw_w * raw_h * 3 * 1.05 + raw_w // 2 * raw_h * 3 * 0.3)
    return int(per_visit * len(config['target_ids']) * config['cycles'])


def git_info():
    def run(*args):
        try:
            return subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.SubprocessError) as error:
            return f'unavailable: {error}'
    diff = run('diff', 'HEAD')
    return {'head': run('rev-parse', 'HEAD').strip(), 'status_porcelain': run('status', '--porcelain'),
            'diff_sha256': hashlib.sha256(diff.encode()).hexdigest()}


def snapshot(session, config, config_path, paths, extra=None):
    """Copy effective config, calibrations, sources and robot model; hash everything."""
    target = Path(session) / 'snapshot'
    target.mkdir()
    import storage
    storage.atomic_json(target / 'effective_config.json', config)
    copied = {}
    for name, path in paths.items():
        if Path(path).is_file():
            destination = target / 'files' / name.replace(':', '__').replace('/', '__')
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, destination)
            copied[name] = {'source': str(path), 'copy': str(destination.relative_to(session)),
                            'sha256': sha256_file(path)}
    sources = target / 'software'
    sources.mkdir()
    software = {}
    for pattern in SOFTWARE_GLOBS:
        for path in sorted(HERE.glob(pattern)):
            if path.is_file():
                destination = sources / path.relative_to(HERE)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, destination)
                software[str(path.relative_to(HERE))] = sha256_file(path)
    environment = {'python': sys.version, 'executable': sys.executable, 'platform': platform.platform(),
                   'hostname': platform.node(),
                   'env': {k: v for k, v in os.environ.items()
                           if k.startswith(('ROS_', 'RMW_', 'AMENT_PREFIX', 'PYTHONPATH', 'LD_LIBRARY'))},
                   'git': git_info(), 'numpy': np.__version__}
    try:
        import cv2
        environment['opencv'] = cv2.__version__
    except ImportError:
        environment['opencv'] = None
    manifest = {'timestamp': utc(), 'config_path': str(config_path), 'files': copied,
                'software_sha256': software, 'environment': environment, **(extra or {})}
    storage.atomic_json(target / 'manifest.json', manifest)
    return manifest
