# Hiwonder xArm ESP32 ros2_control hardware

This package connects `ros2_control` to the binary passthrough Rust firmware
through the existing `HiwonderRustDriver` host driver.

## Calibration model

Each commanded joint uses the following reversible conversion:

```text
joint = ros_reference + direction * (raw - raw_reference) * units_per_raw
```

Revolute joints use `4.1887902047863905 / 1000` radians per raw unit. The
gripper currently uses a linear approximation of `0.03 / 540` metres per raw
unit and will need a separate geometric validation.

The current references come from the pose measured in RViz:

| Servo | Joint | Raw reference | ROS reference | Safe raw range |
|---:|---|---:|---:|---:|
| 1 | `gripper_left_joint` | 443 | 0.0 m | 100–647 (I/O disabled: cable broken) |
| 2 | `limb5_to_limb4_joint` | 512 | -0.008483 rad | 26–959 |
| 3 | `limb4_to_limb3_joint` | 127 | -1.573432 rad | 69–980 |
| 4 | `limb3_to_limb2_joint` | 863 | 1.550816 rad | 20–980 |
| 5 | `limb2_to_limb1_joint` | 856 | 1.550816 rad | 119–849 |
| 6 | `limb1_to_base_link_joint` | 501 | 0.015082 rad | 20–980 |

Every outgoing value is clamped first to the protocol range `0..1000` and
then to the joint's safe raw range. The same converted limits are exposed to
both `ros2_control` and MoveIt.

## Read-only RViz calibration

The following launch uses the real hardware plugin only to read servo IDs 2–6.
It starts `robot_state_publisher`, `joint_state_broadcaster`, and RViz only:
there is no MoveIt process and no arm or gripper controller. `read_only=true`
suppresses all hardware outputs, including deactivation `STOP` frames. The launch does not
address ID 1 at all; its ROS state is fixed to the safe calibrated reference.
After opening the serial port, the plugin waits `startup_delay_ms=2000` before
its first read, allowing a CH340-triggered ESP32 reset to complete.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hiwonder_xarm_esp32_moveit_config calibration_rviz.launch.py \
  serial_port:=/dev/ttyUSB0
```

The raw-to-ROS conversion currently keeps the provisional `direction=+1`
assumption. RViz observations must confirm each direction before a normal
motion launch. ROS/MoveIt bounds for IDs 3, 5, and 6 were recalculated from
that provisional sign; ID 2 remains constrained by the URDF wrist limit and
the unavailable gripper's ROS bounds are intentionally unchanged.

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

The initial `direction` is `+1` for every servo because it matches the
previous RViz pose measurements, but each joint direction must still be
confirmed with a very small inward movement before normal trajectory tests.

Servo 5 was previously observed at raw position `856`, while its safe maximum
is `842`; place it inside the safe range before the first controller test so
that MoveIt starts from a valid state.
