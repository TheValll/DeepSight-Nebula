from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / 'app/calibration/stereo_calib.xml').is_file())
    weights = root / 'benchmarks/finetuning/ping_pong_20261010/yolo26n/weights/best.pt'
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')
    camera_index = LaunchConfiguration('camera_index')
    calibration_file = LaunchConfiguration('calibration_file')
    model = LaunchConfiguration('model')
    confidence = LaunchConfiguration('confidence')
    prompt = LaunchConfiguration('prompt')
    base_vertical_offset = LaunchConfiguration('base_vertical_offset')
    minimum_depth = LaunchConfiguration('minimum_depth')
    num_disparities = LaunchConfiguration('num_disparities')
    grasp_vertical_offset = LaunchConfiguration('grasp_vertical_offset')

    hardware_launch = PathJoinSubstitution(
        [
            FindPackageShare('hiwonder_xarm_esp32_moveit_config'),
            'launch',
            'hardware.launch.py',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('camera_index', default_value='2'),
            DeclareLaunchArgument(
                'calibration_file',
                default_value=str(root / 'app/calibration/stereo_calib_standard.xml')
            ),
            DeclareLaunchArgument(
                'model', default_value=str(weights)
            ),
            DeclareLaunchArgument('confidence', default_value='0.75'),
            DeclareLaunchArgument('prompt', default_value=''),
            DeclareLaunchArgument('base_vertical_offset', default_value='0.0'),
            DeclareLaunchArgument('minimum_depth', default_value='0.06'),
            DeclareLaunchArgument('num_disparities', default_value='224'),
            DeclareLaunchArgument('grasp_vertical_offset', default_value='-0.003'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_launch),
                launch_arguments={
                    'serial_port': serial_port,
                    'use_rviz': use_rviz,
                }.items(),
            ),
            Node(
                package='stereo_camera_calibration',
                executable='rectification_exe',
                parameters=[
                    {
                        'calibration_file': calibration_file,
                        'camera_index': camera_index,
                        'minimum_depth': ParameterValue(minimum_depth, value_type=float),
                        'num_disparities': ParameterValue(num_disparities, value_type=int),
                        'show_debug_windows': False,
                        'publish_depth_color': False,
                        'publish_point_cloud': False,
                        'publish_metrics': True,
                    }
                ],
                output='screen',
            ),
            Node(
                package='stereo_camera_calibration',
                executable='yolo_detection_node',
                parameters=[
                    {
                        'model': model,
                        'confidence': confidence,
                        'imgsz': 1280,
                        'prompt': ParameterValue(prompt, value_type=str),
                        'classes': [0],
                        'minimum_depth': ParameterValue(minimum_depth, value_type=float),
                        'base_vertical_offset': ParameterValue(
                            base_vertical_offset, value_type=float
                        ),
                        'show_window': False,
                        'show_marker_labels': False,
                        'publish_annotated_image': True,
                        'publish_metrics': True,
                    }
                ],
                output='screen',
            ),
            Node(
                package='stereo_camera_calibration',
                executable='ball_goal_preview_node',
                parameters=[
                    {
                        'target_frame': 'base_link',
                        'center_offset_z': ParameterValue(
                            grasp_vertical_offset, value_type=float
                        ),
                        'show_marker_labels': False,
                        'sample_count': 5,
                        'allow_latest_transform': False,
                        'publish_metrics': True,
                    }
                ],
                output='screen',
            ),
            Node(
                package='stereo_camera_calibration',
                executable='ball_pick_node',
                parameters=[
                    {
                        'base_frame': 'base_link',
                        'tip_link': 'gripper_tcp_link',
                        'position_tolerance': 0.003,
                        # Measured on the physical gripper (2026-10-16): fingertips stopped
                        # at the front ball surface and right of the ball centre.
                        'grasp_radial_offset': 0.028,
                        'grasp_lateral_offset': 0.008,
                        # Direct approach, as in the successful physical grasp.
                        'pregrasp_height': 0.0,
                        # Beyond about 0.30 m the servo 1 housing reaches the desk.
                        'max_target_distance': 0.29,
                        'planning_attempts': 2,
                        'allowed_planning_time': 1.0,
                        'velocity_scaling': 0.35,
                        # Gentler acceleration AND deceleration under camera load.
                        'acceleration_scaling': 0.05,
                        'gripper_open_position': 0.013651,
                        'gripper_closed_position': -0.013651,
                        'gripper_max_effort': 10.0,
                    }
                ],
                output='screen',
            ),
        ]
    )
