from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')
    moveit_config = (
        MoveItConfigsBuilder(
            'hiwonder_esp32', package_name='hiwonder_xarm_esp32_moveit_config'
        )
        .robot_description(
            mappings={
                'use_mock_hardware': 'false',
                'serial_port': serial_port,
                'gripper_hardware_io_enabled': 'true',
                'read_only': 'false',
            }
        )
        .to_moveit_configs()
    )

    launch_description = LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/ttyUSB0',
                description='Serial port connected to the ESP32 Rust firmware',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
        ]
    )

    for generated_launch in (
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_rsp_launch(moveit_config),
        generate_move_group_launch(moveit_config),
    ):
        for action in generated_launch.entities:
            launch_description.add_action(action)

    rviz_actions = generate_moveit_rviz_launch(moveit_config).entities
    launch_description.add_action(
        GroupAction(actions=rviz_actions, condition=IfCondition(use_rviz))
    )

    launch_description.add_action(
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                str(moveit_config.package_path / 'config/ros2_controllers.yaml')
            ],
            remappings=[
                ('/controller_manager/robot_description', '/robot_description')
            ],
            output='screen',
        )
    )
    for action in generate_spawn_controllers_launch(moveit_config).entities:
        launch_description.add_action(action)

    launch_description.add_action(Node(
        package='hiwonder_xarm_esp32_moveit_config',
        executable='add_ground',
        # The suction cups raise base_link about 5 mm above the desk
        # (checkerboard measurement, 2026-10-16). A grasp brings the servo 1
        # housing about 7 mm from the desk; the pick node removes the servo sag
        # before the final descent, so keep a 4 mm margin on that housing only.
        parameters=[{
            'floor_height': -0.005,
            'floor_clearance': 0.0,
            # Only the servo 1 housing; padded fingers could not reach the ball.
            'padded_links': ['limb5_link'],
            'link_padding': 0.004,
        }],
        output='screen',
    ))

    return launch_description
