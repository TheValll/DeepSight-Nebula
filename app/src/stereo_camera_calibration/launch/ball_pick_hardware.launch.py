"""Complete physical pipeline: real robot, stereo perception and the pick node.

Nothing moves until /pick_ball (or /approach_ball, /prepare_pick_trial) is called.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

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
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare('hiwonder_xarm_esp32_moveit_config'), 'launch', 'hardware.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=SERIAL),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value=CAMERA),
        DeclareLaunchArgument('calibration_file', default_value=str(root / CALIBRATION)),
        DeclareLaunchArgument('model', default_value=str(root / WEIGHTS)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
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
                'publish_depth_color': False,
                'publish_point_cloud': False,
                'publish_metrics': True,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='yolo_detection_node',
            parameters=[pipeline, {
                'model': ParameterValue(LaunchConfiguration('model'), value_type=str),
                'show_window': False,
                'show_marker_labels': False,
                'publish_annotated_image': True,
                'publish_metrics': True,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='ball_goal_preview_node',
            parameters=[pipeline, {
                'show_marker_labels': False,
                'publish_metrics': True,
            }],
            output='screen',
        ),
        Node(
            package='stereo_camera_calibration',
            executable='ball_pick_node',
            parameters=[pipeline],
            output='screen',
        ),
    ])
