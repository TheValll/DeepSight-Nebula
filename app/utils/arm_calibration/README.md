# Arm self-calibration

Identifies the servo zero references and scales of the arm and the pose of the
camera on the arm, with the checkerboard lying on the desk and the stereo
camera. Use it after a mechanical change (servo horn, camera support, gripper)
or when picks land systematically off target.

The quick camera correction done at every startup (`camera_desk_calibration`)
only fixes the camera tilt and height; this tool calibrates the whole chain.

## What it fits

Every servo follows the HiwonderSystem conversion
`joint = ros_reference + direction * (raw - raw_reference) * units_per_raw`.

| Step | Data | Parameters |
|---|---|---|
| `collect` | 20 arm poses looking at the desk board (raw servo positions + board corners) | - |
| `fit` | the board must stay at one place, flat on the desk | shoulder and elbow `ros_reference`, base/shoulder/elbow/wrist scales, camera mount (6 values) |
| `wrist` | a ball held between the fingers, seen by the stereo camera | wrist `ros_reference` (the camera alone cannot separate it from the camera mount) |
| `apply` | - | writes the values, recomputes the limits, keeps `default` at the same raw positions |

The base zero is not fitted: it is equivalent to turning the whole board. The
gripper and the wrist rotation are not changed.

## Usage

Start the robot, the perception and MoveIt, robot in `default`, checkerboard
(`damier_9x6_25mm_A4.pdf`, 25 mm squares) flat on the desk and fully in view:

```bash
source /opt/ros/jazzy/setup.bash
source app/install/setup.bash
source .venv/bin/activate
ros2 launch stereo_camera_calibration ball_pick_hardware.launch.py
```

In a second terminal, from the project root, with the **system** Python (it needs
`scipy`; do not activate `.venv`):

```bash
source /opt/ros/jazzy/setup.bash
source app/install/setup.bash
python3 app/utils/arm_calibration/calibrate_arm.py collect          # about 3 min, moves the arm
python3 app/utils/arm_calibration/calibrate_arm.py fit              # offline, a few minutes
python3 app/utils/arm_calibration/calibrate_arm.py wrist            # guided: ball held in the gripper
python3 app/utils/arm_calibration/calibrate_arm.py apply --dry-run  # shows the changes
python3 app/utils/arm_calibration/calibrate_arm.py apply
```

Then stop and restart the launch.

- `collect` plans safe poses (lowest arm point at least 45 mm above the desk,
  board fully in view) and drives them through MoveIt at reduced speed. When the
  board is not found in a pose, it tries small wrist and base nudges before
  skipping it. Results go to `app/calibration/arm/<date>/samples.json`.
- `fit` prints the reprojection error of the current model and of the fitted one,
  with leave-one-out validation: each pose is predicted by a model fitted on the
  others. Expect a few pixels; a large validation error means bad data (board
  moved during the sweep, too few poses).
- `wrist` opens the gripper, waits for the ball, closes in small steps until the
  gripper stalls on it, measures it, then reopens the gripper.
- `apply` refuses to run if the configuration changed since `collect`.

## Files written by `apply`

- `hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.ros2_control.xacro`:
  scale factors, `ros_reference`, command limits
- `hiwonder_xarm_esp32_moveit_config/config/joint_limits.yaml`: MoveIt limits
  (10 raw units inside the servo ranges, 3 below the wrist)
- `hiwonder_xarm_esp32_moveit_config/config/hiwonder_esp32.srdf`,
  `initial_positions.yaml`, `stereo_camera_calibration/launch/perception_rviz.launch.py`:
  `default` pose converted to keep the same raw servo positions
- `hiwonder_xarm_esp32_description/urdf/hiwonder_esp32_arm.xacro`: base and wrist limits
- `hiwonder_xarm_esp32_description/urdf/hiwonder_esp32_camera.xacro`: camera mount

It also deletes `app/calibration/camera_desk_correction.yaml`, measured with the
previous model.

## History

`app/calibration/arm/20261102_initial`: first calibration (23 poses). Validation
error 5.7 px instead of 20.3 px with the previous model; wrist zero -6.2 deg
(5.3 mm residual on the held ball).
