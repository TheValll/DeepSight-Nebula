from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_file = LaunchConfiguration("calibration_file")
    camera_index = LaunchConfiguration("camera_index")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")
    prompt = LaunchConfiguration("prompt")
    urdf = PathJoinSubstitution(
        [FindPackageShare("hiwonder_xarm_esp32_description"), "urdf", "hiwonder_esp32.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("stereo_camera_calibration"), "rviz", "perception.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file", default_value="calibration/stereo_calib.xml"
            ),
            DeclareLaunchArgument("camera_index", default_value="2"),
            DeclareLaunchArgument(
                "model",
                default_value="../benchmarks/yolo/models/yoloe-26m-seg.pt",
            ),
            DeclareLaunchArgument("confidence", default_value="0.50"),
            DeclareLaunchArgument("prompt", default_value="ping pong ball"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", urdf]), value_type=str
                        )
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[
                    {
                        "zeros.limb1_to_base_link_joint": 0.048592,
                        "zeros.limb2_to_limb1_joint": 1.148692,
                        "zeros.limb3_to_limb2_joint": 1.140315,
                        "zeros.limb4_to_limb3_joint": -1.759371,
                        "zeros.limb5_to_limb4_joint": -0.008483,
                        "zeros.gripper_left_joint": 0.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="stereo_camera_calibration",
                executable="rectification_exe",
                parameters=[
                    {
                        "calibration_file": calibration_file,
                        "camera_index": camera_index,
                        "show_debug_windows": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="stereo_camera_calibration",
                executable="yolo_detection_node",
                parameters=[
                    {
                        "model": model,
                        "confidence": confidence,
                        "imgsz": 1280,
                        "prompt": prompt,
                        "show_window": False,
                    }
                ],
                output="screen",
            ),
            Node(
                package="stereo_camera_calibration",
                executable="ball_goal_preview_node",
                parameters=[
                    {
                        "target_frame": "base_link",
                        "pregrasp_height": 0.05,
                        "sample_count": 10,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
