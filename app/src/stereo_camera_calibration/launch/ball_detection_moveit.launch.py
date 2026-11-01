"""Fine-tuned ball detection with the real robot in enforced read-only mode.

The real hardware publishes measured joint positions; every servo output is
blocked (read_only) and no arm or gripper command controller is loaded.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)

CALIBRATION = 'app/calibration/stereo_calib_standard.xml'
WEIGHTS = 'benchmarks/finetuning/ping_pong_20261010/yolo26n/weights/best.pt'
CAMERA = '/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0'
SERIAL = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


def generate_launch_description():
    # Works from the source file and from a colcon-installed copy in this repo.
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / CALIBRATION).is_file())
    pipeline = PathJoinSubstitution(
        [FindPackageShare('stereo_camera_calibration'), 'config', 'pipeline.yaml'])
    moveit_config = MoveItConfigsBuilder(
        'hiwonder_esp32', package_name='hiwonder_xarm_esp32_moveit_config'
    ).robot_description(mappings={
        'use_mock_hardware': 'false',
        'serial_port': LaunchConfiguration('serial_port'),
        'read_only': 'true',
    }).to_moveit_configs()

    description = LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=SERIAL),
        DeclareLaunchArgument('camera_device', default_value=CAMERA),
        DeclareLaunchArgument('calibration_file', default_value=str(root / CALIBRATION)),
        DeclareLaunchArgument('model', default_value=str(root / WEIGHTS)),
        SetLaunchConfiguration('allow_trajectory_execution', 'false'),
    ])
    # Pass the SAME real/read-only description to every component. The generic
    # demo launch includes files that rebuild a separate mock URDF instead.
    for generated in (
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_rsp_launch(moveit_config),
        generate_move_group_launch(moveit_config),
        generate_moveit_rviz_launch(moveit_config),
    ):
        for action in generated.entities:
            description.add_action(action)
    description.add_action(Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[str(moveit_config.package_path / 'config/ros2_controllers.yaml')],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    ))
    # Only publish joint states: no arm or gripper command controller is loaded.
    description.add_action(Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'], output='screen',
    ))
    description.add_action(Node(
        package='stereo_camera_calibration', executable='rectification_exe',
        parameters=[pipeline, {
            'calibration_file': ParameterValue(
                LaunchConfiguration('calibration_file'), value_type=str),
            'camera_device': ParameterValue(
                LaunchConfiguration('camera_device'), value_type=str),
            'show_debug_windows': False,
            'publish_depth_color': False,
            'publish_point_cloud': False,
            'publish_metrics': True,
        }], output='screen',
    ))
    description.add_action(Node(
        package='stereo_camera_calibration', executable='yolo_detection_node',
        parameters=[pipeline, {
            'model': ParameterValue(LaunchConfiguration('model'), value_type=str),
            'show_window': False,
            'show_marker_labels': False,
            'publish_annotated_image': True,
            'publish_metrics': True,
        }], output='screen',
    ))
    # Passive: shows the goal the pick node would use, in base_link.
    description.add_action(Node(
        package='stereo_camera_calibration', executable='ball_goal_preview_node',
        parameters=[pipeline, {'show_marker_labels': True}], output='screen',
    ))
    return description
