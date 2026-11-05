"""move_group + floor collision object against an ALREADY RUNNING driver.

Does not start ros2_control, robot_state_publisher, RViz, cameras or perception.
Robot description mappings and add_ground parameters mirror hardware.launch.py.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('hiwonder_esp32', package_name='hiwonder_xarm_esp32_moveit_config')
        .robot_description(mappings={
            'use_mock_hardware': 'false',
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'gripper_hardware_io_enabled': 'true',
            'read_only': 'false',
        })
        .to_moveit_configs()
    )
    description = LaunchDescription()
    for action in generate_move_group_launch(moveit_config).entities:
        description.add_action(action)
    description.add_action(Node(
        package='hiwonder_xarm_esp32_moveit_config',
        executable='add_ground',
        parameters=[{
            'floor_height': -0.005,
            'floor_clearance': 0.0,
            'padded_links': ['limb5_link'],
            'link_padding': 0.004,
        }],
        output='screen',
    ))
    return description
