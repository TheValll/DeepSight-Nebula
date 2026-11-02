# Hiwonder xArm ESP32 ros2_control hardware

This package connects `ros2_control` to the binary passthrough Rust firmware
through the existing `HiwonderRustDriver` host driver.

## Calibration model

Each commanded joint uses the following reversible conversion:

```text
joint = ros_reference + direction * (raw - raw_reference) * units_per_raw
```

The nominal servo scale is `4.1887902047863905 / 1000` radians per raw unit
(240° per 1000 units). Scales and zero references were identified on 2026-11-02
by an arm self-calibration: the arm visited 23 poses looking at a checkerboard
lying on the desk, and the shoulder/elbow zeros, the joint scales and the camera
mount were fitted so that the board stays at the same place (leave-one-out error
5.7 px instead of 20.3 px). The wrist zero comes from a ball held in the gripper
and measured by the stereo camera.

| Servo | Joint | Scale factor | Radians per raw unit |
|---:|---|---:|---:|
| 6 | `limb1_to_base_link_joint` | 0.9477 | 0.0039697 |
| 5 | `limb2_to_limb1_joint` | 0.9938 | 0.0041628 |
| 4 | `limb3_to_limb2_joint` | 1.0318 | 0.0043220 |
| 3 | `limb4_to_limb3_joint` | 0.9722 | 0.0040723 |

The wrist rotation keeps the nominal value. The gripper uses a linear
approximation of `0.03 / 556` metres per raw unit.

Zero references (base and wrist rotation: pose measured in RViz):

| Servo | Joint | Raw reference | ROS reference | Safe raw range |
|---:|---|---:|---:|---:|
| 1 | `gripper_left_joint` | 528 | 0.0 m | 270–786 |
| 2 | `limb5_to_limb4_joint` | 512 | -0.008483 rad | 141–840 |
| 3 | `limb4_to_limb3_joint` | 503 | -0.036851 rad | 61–858 |
| 4 | `limb3_to_limb2_joint` | 863 | 1.515678 rad | 20–980 |
| 5 | `limb2_to_limb1_joint` | 856 | 1.389281 rad | 119–849 |
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
ros2 launch hiwonder_xarm_esp32_moveit_config calibration_rviz.launch.py
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
ros2 launch hiwonder_xarm_esp32_moveit_config hardware.launch.py
```

Both launches default to the stable path of the CH340 adapter,
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`; override it with
`serial_port:=...` if needed.

The plugin reads all six servos and initializes the controller commands from
their actual positions before accepting trajectory commands. During operation,
a stray reply (another servo, or a bad checksum) is skipped until the expected
reply or the 50 ms timeout; a lost servo reply is retried once; the joint then keeps its last position for
up to `max_consecutive_read_failures` cycles (5 by default) before the plugin
reports an error. Reads during activation stay strict. Deactivation
sends a stop command to every servo but does not unload their torque.

## First physical validation

Start with small movements and keep the robot clear of obstacles before
running a complete MoveIt trajectory.
