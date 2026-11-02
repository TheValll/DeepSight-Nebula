"""Perception only, without the robot: URDF with slider joints, stereo depth,
fine-tuned YOLO detections, the passive ball goal and RViz.

The arm pose comes from joint_state_publisher_gui, so the base_link goal is
only meaningful when the sliders match the physical arm.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CALIBRATION = 'app/calibration/stereo_calib.xml'
CORRECTION = 'app/calibration/camera_desk_correction.yaml'
WEIGHTS = 'benchmarks/finetuning/ping_pong_20261010/yolo26n/weights/best.pt'
CAMERA = '/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0'


def generate_launch_description():
    # Works from the source file and from a colcon-installed copy in this repo.
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / CALIBRATION).is_file())
    pipeline = PathJoinSubstitution(
        [FindPackageShare('stereo_camera_calibration'), 'config', 'pipeline.yaml'])
    urdf = PathJoinSubstitution(
        [FindPackageShare('hiwonder_xarm_esp32_description'), 'urdf', 'hiwonder_esp32.urdf.xacro'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('stereo_camera_calibration'), 'rviz', 'perception.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value=CAMERA),
        DeclareLaunchArgument('calibration_file', default_value=str(root / CALIBRATION)),
        DeclareLaunchArgument('model', default_value=str(root / WEIGHTS)),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str)
            }],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            # Initial sliders = the SRDF `default` pose of the arm.
            parameters=[{
                'zeros.limb1_to_base_link_joint': -0.032553170,
                'zeros.limb2_to_limb1_joint': 0.881433418,
                'zeros.limb3_to_limb2_joint': 1.247726428,
                'zeros.limb4_to_limb3_joint': -1.800249745,
                'zeros.limb5_to_limb4_joint': -0.079692433,
                'zeros.gripper_left_joint': 0.0,
            }],
            output='screen',
        ),
        # Joint sliders are not the real pose: only reuse the saved correction.
        Node(
            package='stereo_camera_calibration',
            executable='camera_desk_calibration',
            parameters=[pipeline, {
                'correction_file': str(root / CORRECTION),
                'calibrate': False,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='rectification_exe',
            parameters=[pipeline, {
                'calibration_file': ParameterValue(
                    LaunchConfiguration('calibration_file'), value_type=str),
                'camera_device': ParameterValue(
                    LaunchConfiguration('camera_device'), value_type=str),
                'show_debug_windows': False,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='yolo_detection_node',
            parameters=[pipeline, {
                'model': ParameterValue(LaunchConfiguration('model'), value_type=str),
                'show_window': False,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='ball_goal_preview_node',
            parameters=[pipeline],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
