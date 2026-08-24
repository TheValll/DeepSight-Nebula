from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibration_file = LaunchConfiguration("calibration_file")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")
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
            DeclareLaunchArgument("model", default_value="yolo11s.pt"),
            DeclareLaunchArgument("confidence", default_value="0.15"),
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
                        "zeros.limb1_to_base_link_joint": 0.015082,
                        "zeros.limb2_to_limb1_joint": 1.550816,
                        "zeros.limb3_to_limb2_joint": 1.550816,
                        "zeros.limb4_to_limb3_joint": -1.573432,
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
                        "classes": [32],
                        "show_window": False,
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
