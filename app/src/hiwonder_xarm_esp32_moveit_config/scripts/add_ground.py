#!/usr/bin/env python3
"""Install a collision floor; never send a robot movement command."""
from copy import deepcopy
from math import isfinite

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def add_floor(scene, height=0.0, clearance=0.005):
    """Keep all existing collision rules, allowing only the fixed base contact."""
    if not all(isfinite(v) for v in (height, clearance)) or clearance < 0:
        raise ValueError('Floor height must be finite and clearance nonnegative')
    floor = CollisionObject()
    floor.id = 'ground'
    floor.header.frame_id = 'base_link'
    floor.operation = CollisionObject.ADD
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.10])
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.z = height + clearance - 0.05
    floor.primitives = [box]
    floor.primitive_poses = [pose]
    scene.world.collision_objects = [floor]
    scene.is_diff = True
    scene.robot_state.is_diff = True
    matrix = scene.allowed_collision_matrix
    for name in ('ground', 'base_link'):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(
                enabled=[False] * len(matrix.entry_names)))
    ground = matrix.entry_names.index('ground')
    base = matrix.entry_names.index('base_link')
    # Explicitly forbid contact with every moving link, including old overrides.
    for i, entry in enumerate(matrix.entry_values):
        entry.enabled[ground] = i == base
        matrix.entry_values[ground].enabled[i] = i == base
    return scene


def main():
    rclpy.init()
    node = Node('ground_collision')
    height = float(node.declare_parameter('floor_height', 0.0).value)
    clearance = float(node.declare_parameter('floor_clearance', 0.005).value)
    get = node.create_client(GetPlanningScene, '/get_planning_scene')
    apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    try:
        if not get.wait_for_service(timeout_sec=60) or not apply.wait_for_service(timeout_sec=5):
            raise RuntimeError('MoveIt scene services unavailable; ground NOT installed')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = get.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cannot read current collision rules')
        request = ApplyPlanningScene.Request()
        request.scene = add_floor(deepcopy(future.result().scene), height, clearance)
        future = apply.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError('MoveIt refused ground collision object')
        node.get_logger().info(
            f'Ground installed in base_link: floor z={height:.3f} m, '
            f'clearance={clearance:.3f} m. Only fixed base contact allowed.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
