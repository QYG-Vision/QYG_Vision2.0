# Daedalus ROS Gimbal Axis Design

## Goal

Make the Daedalus ROS2 boundary use the same physical coordinate semantics as
`QYG_sentry_debug`, without copying the real EC's private pitch sign convention
into ROS messages.

## Coordinate Contract

- ROS world and gimbal frames use x forward, y left, and z up.
- `/gimbal_pose` is the orientation from `gimbal_link` to `map`.
- `/rm_gimbal/cmd.yaw` is an absolute ROS yaw command in degrees.
- `/rm_gimbal/cmd.pitch - 90` is an absolute ROS pitch command in degrees.
- Negative planner pitch means physically aiming upward, matching the current
  planner output contract.
- Real EC receive/transmit pitch negations remain confined to the serial bridge;
  they are not applied to Daedalus ROS poses or commands.

## Implementation

Extract a pure Daedalus conversion function that maps ROS yaw and pitch commands
to the Bevy gimbal rotation. Use the same Bevy-to-ROS basis conversion as
`pose_stamped`, so command input and pose output are mathematical inverses at the
ROS boundary. `process_subscription` will update gimbal state and transform using
this function while preserving firing and `distance == -1` behavior.

No changes are required in `SimGimbal`, Solver, Planner, or the real serial
protocol for this correction.

## Verification

1. Add Rust unit tests before changing production code.
2. Verify a yaw-only command changes ROS yaw without changing ROS pitch.
3. Verify a pitch-only command changes ROS pitch without changing ROS yaw.
4. Verify planner pitch below zero produces the defined upward physical motion.
5. Run Daedalus's ROS coordinate tests and build the release binary.
6. With auto aim enabled, publish 80, 90, and 100 degree pitch commands and
   measure `/gimbal_pose`; yaw must remain stable while pitch changes by the
   corresponding signed amount.
7. Re-run `sim_gimbal_pose_test`, `sim_gimbal_command_test`, and the complete
   `QYG_sentry_sim` image/pose/control loop.

## Scope

This change corrects only Daedalus's ROS gimbal command axis mapping. It does not
change camera calibration, target detection, ballistic compensation, serial EC
conventions, vehicle controls, or firing behavior.
