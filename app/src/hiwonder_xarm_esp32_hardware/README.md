# Hiwonder xArm ESP32 ros2_control hardware

This package connects `ros2_control` to the binary passthrough Rust firmware
through the existing `HiwonderRustDriver` host driver.

## Calibration model

Each commanded joint uses the following reversible conversion:

```text
joint = ros_reference + direction * (raw - raw_reference) * units_per_raw
```

Revolute joints use `4.1887902047863905 / 1000` radians per raw unit. The
gripper uses a linear approximation of `0.03 / 556` metres per raw unit.

The current references come from the pose measured in RViz:

| Servo | Joint | Raw reference | ROS reference | Safe raw range |
|---:|---|---:|---:|---:|
| 1 | `gripper_left_joint` | 528 | 0.0 m | 270–786 |
| 2 | `limb5_to_limb4_joint` | 512 | -0.008483 rad | 141–840 |
| 3 | `limb4_to_limb3_joint` | 503 | 0.071130 rad | 61–858 |
| 4 | `limb3_to_limb2_joint` | 863 | 1.550816 rad | 20–980 |
| 5 | `limb2_to_limb1_joint` | 856 | 1.550816 rad | 119–849 |
| 6 | `limb1_to_base_link_joint` | 501 | 0.015082 rad | 20–980 |

Every outgoing value is clamped first to the protocol range `0..1000` and
then to the joint's safe raw range. The same converted limits are exposed to
both `ros2_control` and MoveIt.

## Read-only RViz calibration

The following launch uses the real hardware plugin to read servo IDs 1–6.
It starts `robot_state_publisher`, `joint_state_broadcaster`, and RViz only:
there is no MoveIt process and no arm or gripper controller. `read_only=true`
suppresses all hardware outputs, including deactivation `STOP` frames.
After opening the serial port, the plugin waits `startup_delay_ms=2000` before
its first read, allowing a CH340-triggered ESP32 reset to complete.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hiwonder_xarm_esp32_moveit_config calibration_rviz.launch.py \
  serial_port:=/dev/ttyUSB0
```

The gripper was measured from raw `250` open to `806` closed. Its safe range
is `270..786`. Its named poses use `275..781` to absorb position overshoot,
and its hardware movements use a 1000 ms duration. The continuous wrist was
limited to `141..840` to protect its cable.

The base and wrist use `direction=+1`, with URDF axes `+Z` and `-Y`.
Servo zero offsets are calibrated through `raw_reference` and `ros_reference`;
the fixed URDF geometry is left unchanged.

## Hardware launch

From the `app` workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hiwonder_xarm_esp32_moveit_config hardware.launch.py \
  serial_port:=/dev/ttyUSB0
```

The plugin reads all six servos and initializes the controller commands from
their actual positions before accepting trajectory commands. Deactivation
sends a stop command to every servo but does not unload their torque.

## First physical validation

Start with small movements and keep the robot clear of obstacles before
running a complete MoveIt trajectory.
