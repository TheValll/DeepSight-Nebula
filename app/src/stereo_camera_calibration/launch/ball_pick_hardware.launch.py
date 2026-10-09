from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')
    camera_index = LaunchConfiguration('camera_index')
    calibration_file = LaunchConfiguration('calibration_file')
    model = LaunchConfiguration('model')
    confidence = LaunchConfiguration('confidence')
    prompt = LaunchConfiguration('prompt')

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
                'calibration_file', default_value='calibration/stereo_calib.xml'
            ),
            DeclareLaunchArgument(
                'model', default_value='../benchmarks/yolo/models/yoloe-26m-seg.pt'
            ),
            DeclareLaunchArgument('confidence', default_value='0.50'),
            DeclareLaunchArgument('prompt', default_value='ping pong ball'),
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
                        'prompt': prompt,
                        'show_window': False,
                        'publish_annotated_image': False,
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
                        'max_target_distance': 0.60,
                        'planning_attempts': 2,
                        'allowed_planning_time': 1.0,
                        'velocity_scaling': 0.35,
                        'acceleration_scaling': 0.35,
                        'gripper_open_position': 0.013651,
                        'gripper_closed_position': -0.013651,
                        'gripper_max_effort': 10.0,
                    }
                ],
                output='screen',
            ),
        ]
    )
