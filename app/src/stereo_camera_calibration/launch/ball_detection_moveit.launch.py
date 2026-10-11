"""Fine-tuned ball detection with the real robot in enforced read-only mode."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    # Works from either the source file or a colcon-installed copy in this repo.
    root = next(parent for parent in Path(__file__).resolve().parents
                if (parent / "app/calibration/stereo_calib.xml").is_file())
    weights = root / "benchmarks/finetuning/ping_pong_20261010/yolo26n/weights/best.pt"
    moveit_config = MoveItConfigsBuilder(
        "hiwonder_esp32", package_name="hiwonder_xarm_esp32_moveit_config"
    ).robot_description(mappings={
        "use_mock_hardware": "false",
        "serial_port": LaunchConfiguration("serial_port"),
        "read_only": "true",
    }).to_moveit_configs()
    # The real hardware publishes measured positions, but all outputs are blocked.
    description = LaunchDescription()
    for argument in (
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("camera_index", default_value="2"),
        DeclareLaunchArgument("calibration_file", default_value=str(root / "app/calibration/stereo_calib_standard.xml")),
        DeclareLaunchArgument("model", default_value=str(weights)),
        DeclareLaunchArgument("confidence", default_value="0.50"),
        DeclareLaunchArgument("minimum_depth", default_value="0.06"),
        DeclareLaunchArgument("num_disparities", default_value="224"),
        DeclareLaunchArgument("base_vertical_offset", default_value="0.01"),
    ):
        description.add_action(argument)
    description.add_action(SetLaunchConfiguration("allow_trajectory_execution", "false"))
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
        package="controller_manager", executable="ros2_control_node",
        parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml")],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    ))
    # Only publish joint states: no arm or gripper command controller is loaded.
    description.add_action(Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"], output="screen",
    ))
    description.add_action(Node(
        package="stereo_camera_calibration", executable="rectification_exe",
        parameters=[{
            "camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
            "calibration_file": ParameterValue(LaunchConfiguration("calibration_file"), value_type=str),
            "show_debug_windows": False,
            "publish_depth_color": False,
            "publish_point_cloud": False,
            "publish_metrics": True,
            "minimum_depth": ParameterValue(LaunchConfiguration("minimum_depth"), value_type=float),
            "num_disparities": ParameterValue(LaunchConfiguration("num_disparities"), value_type=int),
        }], output="screen",
    ))
    description.add_action(Node(
        package="stereo_camera_calibration", executable="yolo_detection_node",
        parameters=[{
            "model": ParameterValue(LaunchConfiguration("model"), value_type=str),
            "confidence": ParameterValue(LaunchConfiguration("confidence"), value_type=float),
            "imgsz": 1280,
            "device": ParameterValue("0", value_type=str),
            "classes": [0],
            "prompt": ParameterValue("", value_type=str),
            "show_window": False,
            "show_marker_labels": False,
            "minimum_depth": ParameterValue(LaunchConfiguration("minimum_depth"), value_type=float),
            "base_vertical_offset": ParameterValue(LaunchConfiguration("base_vertical_offset"), value_type=float),
            "publish_annotated_image": True,
            "publish_metrics": True,
        }], output="screen",
    ))
    # No ball_goal_preview, ball_plan_preview or ball_pick node: detection only.
    return description
