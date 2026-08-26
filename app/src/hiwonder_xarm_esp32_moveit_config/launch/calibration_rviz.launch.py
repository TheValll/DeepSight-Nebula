"""Read-only RViz calibration launch for measured joint-state verification.

This launch deliberately starts no MoveIt process and configures no position
controller.  HiwonderSystem receives read_only:=true, so even an accidental
position command cannot result in a servo move.  Servo 1 is additionally
excluded from all hardware I/O because its cable is unavailable.
"""

import os
import stat

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def serial_port_validation_error(serial_port):
    """Return a human-readable error unless *serial_port* is a character device."""
    try:
        mode = os.stat(serial_port).st_mode
    except FileNotFoundError:
        return "serial_port '{}' does not exist".format(serial_port)
    except OSError as error:
        return "cannot stat serial_port '{}': {}".format(serial_port, error)
    if not stat.S_ISCHR(mode):
        return "serial_port '{}' is not a character device".format(serial_port)
    return None


def _validation_failure(message):
    return [
        LogInfo(msg="[calibration_rviz] ERROR: {}".format(message)),
        EmitEvent(event=Shutdown(reason="calibration serial-port validation failed: {}".format(message))),
    ]


def _start_after_serial_validation(context, serial_port, *actions):
    error = serial_port_validation_error(serial_port.perform(context))
    if error:
        return _validation_failure(error)
    return list(actions)


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    config_package = FindPackageShare('hiwonder_xarm_esp32_moveit_config')
    description_package = FindPackageShare('hiwonder_xarm_esp32_description')
    urdf = PathJoinSubstitution([config_package, 'config', 'hiwonder_esp32.urdf.xacro'])
    initial_positions = PathJoinSubstitution(
        [config_package, 'config', 'initial_positions.yaml']
    )
    controllers = PathJoinSubstitution(
        [config_package, 'config', 'calibration_read_only_controllers.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [description_package, 'rviz', 'urdf_config.rviz']
    )
    robot_description = {
        'robot_description': Command(
            [
                FindExecutable(name='xacro'), ' ', urdf,
                ' initial_positions_file:=', initial_positions,
                ' use_mock_hardware:=false',
                ' serial_port:=', serial_port,
                ' gripper_hardware_io_enabled:=false',
                ' read_only:=true',
            ]
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers],
        output='screen',
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    def start_rviz_after_spawner(event, _context):
        if event.returncode == 0:
            return [rviz]
        return _validation_failure(
            'joint_state_broadcaster spawner exited with return code {}'.format(
                event.returncode
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value='/dev/ttyUSB0',
                description='Serial port connected to the ESP32 Rust firmware',
            ),
            # Nodes are created only after this runtime validation succeeds.
            OpaqueFunction(
                function=_start_after_serial_validation,
                args=[
                    serial_port,
                    robot_state_publisher,
                    ros2_control_node,
                    joint_state_spawner,
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=joint_state_spawner,
                            on_exit=start_rviz_after_spawner,
                        )
                    ),
                ],
            ),
        ]
    )
