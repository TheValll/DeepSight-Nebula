#!/usr/bin/env python3
"""Arm self-calibration with the desk checkerboard and the stereo camera.

Run with the system Python (scipy) after sourcing ROS and app/install, while
ball_pick_hardware.launch.py runs, the robot in `default` and the checkerboard
lying flat on the desk in view:

    python3 app/utils/arm_calibration/calibrate_arm.py collect      # moves the arm
    python3 app/utils/arm_calibration/calibrate_arm.py fit
    python3 app/utils/arm_calibration/calibrate_arm.py wrist        # ball held in the gripper
    python3 app/utils/arm_calibration/calibrate_arm.py apply --dry-run
    python3 app/utils/arm_calibration/calibrate_arm.py apply

Model. Every arm joint follows the HiwonderSystem conversion
    joint = ros_reference + direction * (raw - raw_reference) * units_per_raw.
`collect` records raw servo positions and checkerboard corners in many poses.
`fit` finds the shoulder/elbow ros_reference, the four pitch/base scales and the
camera mount (limb4 -> camera) that keep the board at one place, flat on the
desk. The wrist zero cannot be separated from the camera mount by the camera
alone: `wrist` measures a ball held between the fingers to get it. `apply`
writes the values into the configuration, keeping the `default` pose at the
same raw servo positions.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import re
import sys
import time

import numpy as np
import yaml

try:
    from scipy.optimize import least_squares, minimize_scalar
    from scipy.spatial.transform import Rotation
except ImportError:
    sys.exit('scipy is missing: run with the system python3, not the .venv one.')

from kinematics import (ARM, CAMERA_FRAME, CAMERA_MOUNT_LINK, NOMINAL_RADIANS_PER_RAW, TCP_FRAME,
                        Robot, axis_rotation, homogeneous)

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / 'app/src/hiwonder_xarm_esp32_moveit_config/config'
PIPELINE = ROOT / 'app/src/stereo_camera_calibration/config/pipeline.yaml'
OUTPUT = ROOT / 'app/calibration/arm'
BOARD_PATTERN = (8, 5)
SQUARE = 0.025
GRID = np.c_[np.mgrid[0:8, 0:5].T.reshape(-1, 2) * SQUARE, np.zeros(40), np.ones(40)]
FLIP = np.diag([1.0, -1.0, -1.0])        # OpenCV board z points into the desk
FITTED_REFERENCES = ARM[1:3]             # shoulder, elbow zeros (base: gauge, wrist: `wrist`)
FITTED_SCALES = ARM[:4]
URDF_CLAMPED = {'limb2_to_limb1_joint', 'limb3_to_limb2_joint'}
WRIST_CHILD = 'limb4_link'               # child link of limb4_to_limb3_joint


def desk_height():
    return yaml.safe_load(PIPELINE.read_text())['camera_desk_calibration']['ros__parameters']['desk_height']


def latest_run():
    runs = sorted(OUTPUT.glob('*/samples.json'))
    if not runs:
        sys.exit(f'No calibration run in {OUTPUT}: run `collect` first.')
    return runs[-1].parent


# --------------------------------------------------------------------------- model

class Model:
    """Calibration parameters around the configuration used during `collect`."""

    def __init__(self, robot, snapshot):
        self.robot = robot
        self.snapshot = snapshot                     # joint -> calibration dict
        self.mount0 = np.array(snapshot['camera_mount'])

    def initial(self):
        cal = self.snapshot['joints']
        return np.r_[[cal[j]['ros_reference'] for j in FITTED_REFERENCES],
                     [cal[j]['units_per_raw'] / NOMINAL_RADIANS_PER_RAW for j in FITTED_SCALES],
                     np.zeros(6)]

    def joints(self, raw, p):
        cal = self.snapshot['joints']
        refs = dict(zip(FITTED_REFERENCES, p[0:2]))
        factors = dict(zip(FITTED_SCALES, p[2:6]))
        out = {}
        for name, value in raw.items():
            c = cal[name]
            upr = factors[name] * NOMINAL_RADIANS_PER_RAW if name in factors else c['units_per_raw']
            out[name] = refs.get(name, c['ros_reference']) + c['direction'] * (value - c['raw_reference']) * upr
        return out

    def mount(self, p):
        return self.mount0 @ homogeneous(Rotation.from_rotvec(p[9:12]).as_matrix(), p[6:9])

    def camera(self, raw, p):
        return self.robot.transform(CAMERA_FRAME, self.joints(raw, p),
                                    overrides={CAMERA_MOUNT_LINK: self.mount(p)})


def board_pose(b, z):
    return homogeneous(Rotation.from_euler('z', b[2]).as_matrix() @ FLIP, [b[0], b[1], z])


def reprojection(model, sample, p, board):
    pc = (np.linalg.inv(model.camera(sample['raw'], p)) @ board @ GRID.T).T[:, :3]
    uv = (np.array(sample['K']) @ (pc / pc[:, 2:3]).T).T[:, :2]
    return (uv - np.array(sample['corners'])).ravel()


def fit_samples(model, samples, z, free, p_start, b_start):
    """Least squares on the free parameters (bool mask over the 12) + board x, y, yaw."""
    def residuals(x):
        p = p_start.copy()
        p[free] = x[:free.sum()]
        board = board_pose(x[free.sum():], z)
        return np.concatenate([reprojection(model, s, p, board) for s in samples])
    result = least_squares(residuals, np.r_[p_start[free], b_start], x_scale='jac',
                           loss='soft_l1', f_scale=2.0)
    p = p_start.copy()
    p[free] = result.x[:free.sum()]
    return p, result.x[free.sum():]


def rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


# --------------------------------------------------------------------------- collect

def collect(args):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from tf2_ros import Buffer, TransformListener

    robot = Robot()
    rclpy.init()
    node = rclpy.create_node('arm_self_calibration')
    bridge = CvBridge()
    tf = Buffer()
    TransformListener(tf, node)
    state = {'joints': {}, 'image': None, 'k': None}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state['joints'].update(zip(m.name, m.position)), 10)
    node.create_subscription(Image, '/stereo/left/image_rect',
                             lambda m: state.update(image=m), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/stereo/depth/camera_info',
                             lambda m: state.update(k=m.k), qos_profile_sensor_data)
    move = ActionClient(node, MoveGroup, '/move_action')
    if not move.wait_for_server(timeout_sec=10):
        sys.exit('MoveIt /move_action is not available: start ball_pick_hardware.launch.py.')

    def spin_for(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def camera_matrix():
        k = state['k']
        return np.array([[2 * k[0], 0, 2 * k[2] + 0.5], [0, 2 * k[4], 2 * k[5] + 0.5], [0, 0, 1]])

    def capture(count=10, timeout=6.0):
        spin_for(2.0)                                   # let the arm settle
        corners, joints, image = [], [], None
        end = time.time() + timeout
        while time.time() < end and len(corners) < count:
            state['image'] = None
            while state['image'] is None and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if state['image'] is None:
                break
            image = state['image']
            gray = cv2.cvtColor(bridge.imgmsg_to_cv2(image, 'bgr8'), cv2.COLOR_BGR2GRAY)
            found, c = cv2.findChessboardCornersSB(gray, BOARD_PATTERN)
            if found:
                corners.append(c.reshape(-1, 2))
                joints.append(dict(state['joints']))
        return corners, joints, image

    def go(pose):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.35
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name, value in pose.items():
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value), tolerance_above=0.01,
                tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        future = move.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        if handle is None or not handle.accepted:
            return 'refused'
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=60)
        return result.result().result.error_code.val if result.done() else 'timeout'

    spin_for(2.0)
    if state['k'] is None:
        sys.exit('No /stereo/depth/camera_info: is the perception running?')
    K = camera_matrix()

    # 1. Board location from the current pose (URDF camera frame, no desk correction).
    corners, joints, image = capture()
    if len(corners) < 5:
        sys.exit('Checkerboard not seen from the current pose: put it flat on the desk, in view.')
    ok, rvec, tvec = cv2.solvePnP(GRID[:, :3].astype(np.float32), np.median(corners, axis=0)
                                  .astype(np.float32), K, None)
    t = tf.lookup_transform('base_link', CAMERA_FRAME, Time.from_msg(image.header.stamp),
                            timeout=Duration(seconds=1.0))
    q = t.transform.rotation
    base_from_camera = homogeneous(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                   [t.transform.translation.x, t.transform.translation.y,
                                    t.transform.translation.z])
    board = (base_from_camera @ homogeneous(cv2.Rodrigues(rvec)[0], tvec.ravel()) @ GRID.T).T[:, :3]
    print(f'Damier vu, centre base_link {np.round(board.mean(0) * 1000)} mm', flush=True)

    # 2. Plan diverse, safe poses that keep the board in view.
    limits = yaml.safe_load((CONFIG / 'joint_limits.yaml').read_text())['joint_limits']
    start = {name: state['joints'][name] for name in ARM}
    probes = [np.array([-0.0013, -0.1085, 0.0007]) + s * 0.03 * np.array([0.423, 0, 0.906])
              for s in (-1, 1)] + [np.zeros(3), np.array([0.0, -0.06, 0.0])]

    def predicted(q):
        T = robot.transform(CAMERA_FRAME, q)
        pc = (np.linalg.inv(T) @ np.c_[board, np.ones(40)].T).T[:, :3]
        if (pc[:, 2] < 0.12).any():
            return None
        uv = (K @ (pc / pc[:, 2:3]).T).T[:, :2]
        if (uv[:, 0] < 90).any() or (uv[:, 0] > 1190).any() or (uv[:, 1] < 90).any() or (uv[:, 1] > 630).any():
            return None
        normal = T[:3, :3].T @ np.array([0, 0, 1.0])
        view = math.degrees(math.acos(abs(normal @ (pc.mean(0) / np.linalg.norm(pc.mean(0))))))
        if view > 55:
            return None
        T5 = robot.transform('limb5_link', q)
        low = min((T5 @ np.r_[p, 1])[2] for p in probes)
        low = min([low] + [robot.transform(link, q)[2, 3]
                           for link in ('limb4_link', 'camera_cover_link', 'limb3_link')])
        return None if low < 0.045 else low

    rng = np.random.default_rng(args.seed)
    candidates = []
    for _ in range(args.samples):
        q = dict(start)
        q[ARM[0]] = start[ARM[0]] + rng.uniform(-0.45, 0.45)
        for name in ARM[1:4]:
            q[name] = rng.uniform(limits[name]['min_position'] + 0.05, limits[name]['max_position'] - 0.05)
        if predicted(q):
            candidates.append(q)
    if len(candidates) < args.poses:
        sys.exit(f'Only {len(candidates)} safe poses keep the board in view: move the board.')
    X = np.array([[c[n] for n in ARM[:4]] for c in candidates])
    chosen = [int(np.argmin(np.linalg.norm(X - [start[n] for n in ARM[:4]], axis=1)))]
    distance = np.linalg.norm(X - X[chosen[0]], axis=1)
    while len(chosen) < args.poses:
        index = int(np.argmax(distance))
        chosen.append(index)
        distance = np.minimum(distance, np.linalg.norm(X - X[index], axis=1))
    order, rest = [chosen[0]], chosen[1:]
    while rest:
        nxt = min(rest, key=lambda i: np.linalg.norm(X[i] - X[order[-1]]))
        order.append(nxt)
        rest.remove(nxt)
    poses = [candidates[i] for i in order]
    print(f'{len(candidates)} poses sûres ; {len(poses)} retenues', flush=True)

    # 3. Sweep, with small nudges when the board is not found.
    run = OUTPUT / datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True)
    snapshot = {'joints': {n: robot.calibration[n].as_dict() for n in ARM},
                'camera_mount': robot.camera_mount.tolist()}
    samples = []
    for index, pose in enumerate(poses + [start]):
        final = index == len(poses)
        label = 'retour au départ' if final else f'pose {index + 1}/{len(poses)}'
        code = go(pose)
        if code != MoveItErrorCodes.SUCCESS:
            print(f'{label}: mouvement refusé ou échoué ({code})', flush=True)
            if code in ('refused', 'timeout') or code == MoveItErrorCodes.CONTROL_FAILED:
                break
            continue
        if final:
            print(f'{label}: ok', flush=True)
            break
        corners, joints, _ = capture()
        for name, delta in (('limb4_to_limb3_joint', 0.12), ('limb4_to_limb3_joint', -0.12),
                            ('limb1_to_base_link_joint', 0.10), ('limb1_to_base_link_joint', -0.10)):
            if len(corners) >= 5:
                break
            nudged = dict(pose)
            nudged[name] += delta
            code = go(nudged)
            print(f'{label}: damier non vu, ajustement {name.split("_to_")[0]} {delta:+.2f} rad '
                  f'-> {"ok" if code == MoveItErrorCodes.SUCCESS else code}', flush=True)
            if code == MoveItErrorCodes.SUCCESS:
                corners, joints, _ = capture()
        if len(corners) < 5:
            print(f'{label}: damier non vu, pose ignorée', flush=True)
            continue
        spread = float(np.std(np.array(corners), axis=0).max())
        if spread > 1.0:
            print(f'{label}: coins instables ({spread:.1f} px), pose ignorée', flush=True)
            continue
        raw = {n: float(robot.calibration[n].to_raw(np.median([j[n] for j in joints]))) for n in ARM}
        samples.append({'raw': raw, 'corners': np.median(corners, axis=0).tolist(),
                        'K': K.tolist(), 'corner_spread_px': spread})
        print(f'{label}: ok ({len(corners)} images, {spread:.2f} px)', flush=True)
        (run / 'samples.json').write_text(json.dumps({'snapshot': snapshot, 'samples': samples}))
    print(f'{len(samples)} poses enregistrées dans {run}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


# --------------------------------------------------------------------------- fit

def fit(args):
    run = Path(args.run) if args.run else latest_run()
    data = json.loads((run / 'samples.json').read_text())
    samples, z = data['samples'], desk_height()
    if len(samples) < 12:
        sys.exit(f'Only {len(samples)} poses: collect at least 12.')
    model = Model(Robot(), data['snapshot'])
    p0 = model.initial()
    # Initial board: first view through the current model.
    import cv2
    s0 = samples[0]
    ok, rvec, tvec = cv2.solvePnP(GRID[:, :3].astype(np.float32),
                                  np.array(s0['corners'], np.float32), np.array(s0['K']), None)
    T0 = model.camera(s0['raw'], p0) @ homogeneous(cv2.Rodrigues(rvec)[0], tvec.ravel())
    axis_x = T0[:3, 0]
    b0 = np.array([T0[0, 3], T0[1, 3], math.atan2(axis_x[1], axis_x[0])])

    free_all = np.ones(12, bool)
    report = {}
    for label, free in (('modèle actuel', np.zeros(12, bool)), ('calibration', free_all)):
        p, b = fit_samples(model, samples, z, free, p0, b0)
        train = rms(np.concatenate([reprojection(model, s, p, board_pose(b, z)) for s in samples]))
        loo = []
        for i in range(len(samples)):
            pi, bi = fit_samples(model, samples[:i] + samples[i + 1:], z, free, p, b)
            loo.append(rms(reprojection(model, samples[i], pi, board_pose(bi, z))))
        report[label] = {'train_px': train, 'loo_px': rms(loo), 'loo_worst_px': max(loo)}
        print(f'{label:14s} ajusté {train:5.1f} px | validation croisée {rms(loo):5.1f} px '
              f'(pire pose {max(loo):.1f})', flush=True)
    cal = data['snapshot']['joints']
    result = {
        'run': str(run), 'samples': len(samples), 'report': report,
        'ros_reference': {**{n: cal[n]['ros_reference'] for n in ARM}, **dict(zip(FITTED_REFERENCES, p[0:2]))},
        'scale_factor': {**{n: cal[n]['units_per_raw'] / NOMINAL_RADIANS_PER_RAW for n in ARM},
                         **dict(zip(FITTED_SCALES, p[2:6]))},
        'camera_mount': model.mount(p).tolist(),
        'board': b.tolist(),
    }
    for n in ARM[:4]:
        print(f'  {n:26s} ros_reference {cal[n]["ros_reference"]:+.6f} -> {result["ros_reference"][n]:+.6f}'
              f' | facteur {cal[n]["units_per_raw"] / NOMINAL_RADIANS_PER_RAW:.4f} -> {result["scale_factor"][n]:.4f}')
    print('  support caméra :', 'xyz', np.round(model.mount(p)[:3, 3], 4), 'rpy',
          np.round(Rotation.from_matrix(model.mount(p)[:3, :3]).as_euler('xyz'), 4))
    (run / 'fit.json').write_text(json.dumps(result, indent=1))
    print(f'Résultat dans {run / "fit.json"}. Étape suivante : `wrist` (balle tenue dans la pince).')


# --------------------------------------------------------------------------- wrist

def wrist(args):
    import rclpy
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import MarkerArray

    run = Path(args.run) if args.run else latest_run()
    fitted = json.loads((run / 'fit.json').read_text())
    robot = Robot()
    rclpy.init()
    node = rclpy.create_node('arm_calibration_wrist')
    state = {'joints': {}, 'balls': []}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state['joints'].update(zip(m.name, m.position)), 10)

    def on_markers(msg):
        balls = [m.pose.position for m in msg.markers if m.ns == 'detected_objects']
        if balls:
            b = min(balls, key=lambda p: p.z)
            state['balls'].append([b.x, b.y, b.z])

    node.create_subscription(MarkerArray, '/yolo/markers', on_markers, qos_profile_sensor_data)
    gripper = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
    if not gripper.wait_for_server(timeout_sec=10):
        sys.exit('Gripper controller not available: start ball_pick_hardware.launch.py.')

    def command(position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        future = gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=10)
        return result.result().result

    command(0.013651)
    input('Pince ouverte. Place la balle entre les doigts, tiens-la, puis appuie sur Entrée...')
    command(0.008)
    position = 0.006
    while position > -0.012:
        r = command(position)
        print(f'  consigne {position:+.4f} -> pince {r.position:+.4f}{" (bloquée sur la balle)" if r.stalled else ""}')
        if r.stalled:
            break
        position -= 0.0015
    input('Lâche la balle (la pince la tient). Appuie sur Entrée pour mesurer...')
    state['balls'].clear()
    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(state['balls']) < 10:
        sys.exit('Balle tenue non détectée par la caméra.')
    measured = np.median(np.array(state['balls']), axis=0)
    roll = state['joints']['limb5_to_limb4_joint']
    command(0.013651)
    print('Mesure faite, pince rouverte : récupère la balle.')
    node.destroy_node()
    rclpy.shutdown()

    mount = np.array(fitted['camera_mount'])
    T4_tcp = robot.transform(TCP_FRAME, {'limb5_to_limb4_joint': roll}, stop='limb4_link')
    T4_cam = robot.transform(CAMERA_FRAME, {}, stop='limb4_link', overrides={CAMERA_MOUNT_LINK: mount})
    axis = robot.joint_axis(WRIST_CHILD)

    def predicted(delta):
        rotation = homogeneous(axis_rotation(axis, delta), np.zeros(3))
        return (np.linalg.inv(T4_cam) @ rotation @ T4_tcp)[:3, 3]

    best = minimize_scalar(lambda d: np.linalg.norm(predicted(d) - measured),
                           bounds=(-0.6, 0.6), method='bounded')
    delta = float(best.x)
    residual = float(np.linalg.norm(predicted(delta) - measured))
    print(f'Balle tenue (repère caméra) {np.round(measured * 1000, 1)} mm ; sans correction écart '
          f'{np.linalg.norm(predicted(0.0) - measured) * 1000:.1f} mm')
    print(f'Zéro du poignet {math.degrees(delta):+.2f} deg -> écart restant {residual * 1000:.1f} mm')
    # Same camera pose: the wrist rotation moves from the mount to the joint.
    rotation = homogeneous(axis_rotation(axis, delta), np.zeros(3))
    fitted['camera_mount'] = (np.linalg.inv(rotation) @ mount).tolist()
    fitted['ros_reference']['limb4_to_limb3_joint'] += delta
    fitted['wrist'] = {'offset_rad': delta, 'residual_mm': residual * 1000,
                       'held_ball_camera_m': measured.tolist(), 'roll_rad': roll}
    (run / 'fit.json').write_text(json.dumps(fitted, indent=1))
    print(f'Mis à jour : {run / "fit.json"}. Étape suivante : `apply --dry-run`.')


# --------------------------------------------------------------------------- apply

def apply(args):
    run = Path(args.run) if args.run else latest_run()
    fitted = json.loads((run / 'fit.json').read_text())
    if 'wrist' not in fitted and not args.without_wrist:
        sys.exit('No wrist measurement in fit.json: run `wrist` first (or --without-wrist).')
    robot = Robot()
    snapshot = json.loads((run / 'samples.json').read_text())['snapshot']
    changed = [n for n in ARM if robot.calibration[n].as_dict() != snapshot['joints'][n]]
    if changed:
        sys.exit(f'The configuration changed since `collect` ({changed}): collect again.')

    new = {}
    for name in ARM[:4]:
        c = robot.calibration[name]
        factor = fitted['scale_factor'][name]
        reference = fitted['ros_reference'][name]
        upr = factor * NOMINAL_RADIANS_PER_RAW

        def rad(raw, c=c, reference=reference, upr=upr):
            return reference + c.direction * (raw - c.raw_reference) * upr
        low_margin = 3 if name == 'limb4_to_limb3_joint' else 10
        command = sorted([rad(c.raw_min - 5), rad(c.raw_max + 5)])
        moveit = sorted([rad(c.raw_min + low_margin), rad(c.raw_max - 10)])
        if name in URDF_CLAMPED:
            lo, hi = robot.limits[name]
            command = [max(command[0], lo), min(command[1], hi)]
            moveit = [max(moveit[0], lo + 0.02), min(moveit[1], hi - 0.02)]
        new[name] = {'factor': factor, 'reference': reference, 'command': command,
                     'moveit': moveit, 'rad': rad, 'old': c}

    srdf_path = CONFIG / 'hiwonder_esp32.srdf'
    srdf = srdf_path.read_text()
    default = {}
    for name in ARM[:4]:
        match = re.search(rf'<group_state name="default" group="arm">.*?<joint name="{name}" value="([-0-9.e]+)"',
                          srdf, re.S)
        old_value = float(match.group(1))
        default[name] = (old_value, new[name]['rad'](new[name]['old'].to_raw(old_value)))
    mount = np.array(fitted['camera_mount'])
    mount_xyz = mount[:3, 3]
    mount_rpy = Rotation.from_matrix(mount[:3, :3]).as_euler('xyz')

    print('Joint                      ros_reference            facteur          default (même raw)')
    for name in ARM[:4]:
        n, (d_old, d_new) = new[name], default[name]
        print(f'{name:26s} {n["old"].ros_reference:+.6f} -> {n["reference"]:+.6f}  '
              f'{n["old"].units_per_raw / NOMINAL_RADIANS_PER_RAW:.4f} -> {n["factor"]:.4f}  '
              f'{d_old:+.5f} -> {d_new:+.5f}')
    print('camera_mount xyz', np.round(mount_xyz, 4), 'rpy', np.round(mount_rpy, 4))
    if args.dry_run:
        print('--dry-run : aucun fichier modifié.')
        return

    def edit(path, pattern, replacement, flags=0):
        text = path.read_text()
        updated, count = re.subn(pattern, replacement, text, flags=flags)
        if count == 0:
            sys.exit(f'Pattern not found in {path}: {pattern}')
        path.write_text(updated)

    xacro = CONFIG / 'hiwonder_esp32.ros2_control.xacro'
    properties = {'limb1_to_base_link_joint': 'base', 'limb2_to_limb1_joint': 'shoulder',
                  'limb3_to_limb2_joint': 'elbow', 'limb4_to_limb3_joint': 'wrist'}
    for name, prop in properties.items():
        n = new[name]
        edit(xacro, rf'(<xacro:property name="{prop}_radians_per_raw" value="\$\{{radians_per_raw \* )[0-9.]+(\}}"/>)',
             rf'\g<1>{n["factor"]:.4f}\g<2>')
        edit(xacro, rf'(<joint name="{name}">.*?<param name="ros_reference">)[^<]*(</param>)',
             rf'\g<1>{n["reference"]:.6f}\g<2>', re.S)
        edit(xacro, rf'(<joint name="{name}">.*?<command_interface name="position">\s*<param name="min">)[^<]*'
                    rf'(</param>\s*<param name="max">)[^<]*(</param>)',
             rf'\g<1>{n["command"][0]:.6f}\g<2>{n["command"][1]:.6f}\g<3>', re.S)
        edit(CONFIG / 'joint_limits.yaml', rf'({name}:\n(?:    .*\n)*?    min_position: )[-0-9.]+',
             rf'\g<1>{n["moveit"][0]:.6f}')
        edit(CONFIG / 'joint_limits.yaml', rf'({name}:\n(?:    .*\n)*?    max_position: )[-0-9.]+',
             rf'\g<1>{n["moveit"][1]:.6f}')
        d_old, d_new = default[name]
        edit(srdf_path, rf'(<group_state name="default" group="arm">.*?<joint name="{name}" value=")[-0-9.e]+(")',
             rf'\g<1>{d_new:.9f}\g<2>', re.S)
        edit(CONFIG / 'initial_positions.yaml', rf'(  {name}: )[-0-9.e]+', rf'\g<1>{d_new:.9f}')
        edit(ROOT / 'app/src/stereo_camera_calibration/launch/perception_rviz.launch.py',
             rf"('zeros\.{name}': )[-0-9.e]+", rf'\g<1>{d_new:.9f}')
    arm_xacro = ROOT / 'app/src/hiwonder_xarm_esp32_description/urdf/hiwonder_esp32_arm.xacro'
    for name in ('limb1_to_base_link_joint', 'limb4_to_limb3_joint'):
        lo, hi = new[name]['command']
        edit(arm_xacro, rf'(<joint name="{name}" type="revolute">.*?<limit lower=")[-0-9.]+(" upper=")[-0-9.]+(")',
             rf'\g<1>{lo:.6f}\g<2>{hi:.6f}\g<3>', re.S)
    camera_xacro = ROOT / 'app/src/hiwonder_xarm_esp32_description/urdf/hiwonder_esp32_camera.xacro'
    edit(camera_xacro, r'(<xacro:property name="camera_mount_xyz" value=")[^"]*(")',
         rf'\g<1>{" ".join(f"{v:.4f}" for v in mount_xyz)}\g<2>')
    edit(camera_xacro, r'(<xacro:property name="camera_mount_rpy" value=")[^"]*(")',
         rf'\g<1>{" ".join(f"{v:.4f}" for v in mount_rpy)}\g<2>')
    stale = ROOT / 'app/calibration/camera_desk_correction.yaml'
    if stale.exists():
        stale.unlink()
    print('Configuration mise à jour. Relancer le pipeline : la correction de bureau sera refaite.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('collect', help='move through poses and record the board (moves the arm)')
    p.add_argument('--poses', type=int, default=20)
    p.add_argument('--samples', type=int, default=60000, help='random candidate poses')
    p.add_argument('--seed', type=int, default=7)
    for name in ('fit', 'wrist', 'apply'):
        p = sub.add_parser(name)
        p.add_argument('--run', help='run folder (default: latest in app/calibration/arm)')
        if name == 'apply':
            p.add_argument('--dry-run', action='store_true')
            p.add_argument('--without-wrist', action='store_true')
    args = parser.parse_args()
    {'collect': collect, 'fit': fit, 'wrist': wrist, 'apply': apply}[args.command](args)


if __name__ == '__main__':
    main()
