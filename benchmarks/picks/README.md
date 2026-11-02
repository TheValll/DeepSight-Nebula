# Pick benchmark — 2 November 2026

File: `benchmark_20261102_190826.csv`.

Ball on the desk, 3 directions (right -30°, centre 0°, left +30° from straight ahead)
x 4 horizontal distances from the base (20, 23, 25, 27 cm) x 3 trials = 36 picks.
Each pick: default -> open -> reach -> close -> back to default.

## Result (visual check)

| | Picks |
|---|---:|
| Executed | 34 |
| Ball held after the return to default | **33** |
| Missed (visual) | 1 — one of the 3 trials at right 25 cm |
| Software failure | 1 — right 20 cm trial 1: gripper opened to 0.0131 instead of 0.01365, rejected by the 0.5 mm goal tolerance |
| Refused | 1 — left 27 cm trial 2: target at 0.291 m, above the 0.290 m safety limit |

**33/34 executed picks held the ball** (97 %).

The `verdict` column of the CSV is **not reliable**: it was computed from the gripper
position, but with the gentle closing (target -0.008, Patafix pads) a held ball and
an empty gripper both end between -0.004 and -0.008. Use the visual result above.

## Configuration

- Arm self-calibration of 2026-11-02 (`app/calibration/arm/20261102_initial`), camera
  tilt and height corrected on the desk checkerboard at startup.
- Stereo calibration with the 25 mm board (`damier_9x6_25mm_A4.pdf`), baseline 59.4 mm.
- TCP at the CAD midpoint of the finger pads; Patafix on both fingertips (the fingers
  form a V that opens toward the tips).
- `pipeline.yaml`: `surface_to_center_depth` 0.0055, `position_tolerance` 0.001,
  `grasp_radial_offset` 0.008, `grasp_lateral_offset` 0.003,
  `gripper_closed_position` -0.008; gripper closing in 250 ms.
