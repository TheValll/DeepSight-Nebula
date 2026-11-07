"""Complete physical pipeline: real robot, stereo perception and the pick node.

Nothing moves until /pick_ball (or /approach_ball, /prepare_pick_trial) is called.
"""
from glob import glob
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CALIBRATION = 'app/calibration/stereo_calib.xml'
CORRECTION = 'app/calibration/camera_desk_correction.yaml'
WEIGHTS = 'benchmarks/finetuning/ping_pong_20261010/yolo26n/weights/best.pt'
CAMERA = '0'
SERIAL_PORTS = sorted(glob('/dev/cu.usbserial*') + glob('/dev/cu.usbmodem*'))
SERIAL = SERIAL_PORTS[0] if len(SERIAL_PORTS) == 1 else ''


def generate_launch_description():
    openmp_path = str(
        Path(os.environ.get('HOMEBREW_PREFIX', '/opt/homebrew')) / 'opt/libomp/lib')
    library_path = ':'.join(filter(None, [openmp_path, os.environ.get('DYLD_LIBRARY_PATH')]))
    # Works from the source file and from a colcon-installed copy in this repo.
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / CALIBRATION).is_file())
    pipeline = PathJoinSubstitution(
        [FindPackageShare('stereo_camera_calibration'), 'config', 'pipeline.yaml'])
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare('hiwonder_xarm_esp32_moveit_config'), 'launch', 'hardware.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTDDS_BUILTIN_TRANSPORTS',
            os.environ.get('FASTDDS_BUILTIN_TRANSPORTS')
            or 'LARGE_DATA?max_msg_size=4MB&sockets_size=4MB'),
        SetEnvironmentVariable('DYLD_LIBRARY_PATH', library_path),
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
        # Recalibrates the camera tilt and height on the checkerboard at startup.
        Node(
            package='stereo_camera_calibration',
            executable='camera_desk_calibration',
            parameters=[pipeline, {
                'correction_file': str(root / CORRECTION),
                'calibrate': True,
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
