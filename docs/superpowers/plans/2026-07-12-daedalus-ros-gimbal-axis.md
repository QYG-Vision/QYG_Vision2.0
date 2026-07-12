# Daedalus ROS Gimbal Axis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Daedalus ROS yaw and pitch commands produce matching yaw and pitch changes in `/gimbal_pose`.

**Architecture:** Add an inverse rotation-basis helper beside the existing Bevy-to-ROS transform, then add a pure gimbal-command conversion that includes the existing fixed muzzle alignment. Keep ROS semantics at the ROS boundary and leave the real EC sign conversions in `rm_vision_2025` unchanged.

**Tech Stack:** Rust 2024, Bevy 0.19 `Quat`/`Mat3`, ROS2 Humble through `r2r`, Cargo tests, ROS2 CLI dynamic verification.

---

### Task 1: Inverse ROS Rotation Basis

**Files:**
- Modify: `/home/robomaster/bevy_robomaster_simulator/src/ros2/prelude.rs`
- Test: `/home/robomaster/bevy_robomaster_simulator/src/ros2/prelude.rs`

- [ ] **Step 1: Write the failing round-trip test**

Add this test to the existing `tests` module:

```rust
#[test]
fn ros_rotation_to_bevy_round_trips_through_transform() {
    let ros_rotation = Quat::from_euler(
        EulerRot::ZYX,
        20.0_f32.to_radians(),
        -10.0_f32.to_radians(),
        0.0,
    );
    let bevy_rotation = ros_rotation_to_bevy(ros_rotation);
    let transformed = transform(Transform::from_rotation(bevy_rotation));
    let actual = Quat::from_xyzw(
        transformed.rotation.x as f32,
        transformed.rotation.y as f32,
        transformed.rotation.z as f32,
        transformed.rotation.w as f32,
    );
    assert!(actual.dot(ros_rotation).abs() > 1.0 - 1e-6);
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cargo test --no-default-features --features ros2 ros_rotation_to_bevy_round_trips_through_transform
```

Expected: compilation fails because `ros_rotation_to_bevy` does not exist.

- [ ] **Step 3: Implement the inverse basis conversion**

Add beside `transform`:

```rust
#[inline]
pub fn ros_rotation_to_bevy(ros_rotation: Quat) -> Quat {
    let align_quat = Quat::from_mat3(&M_ALIGN_MAT3);
    align_quat.inverse() * ros_rotation * align_quat
}
```

- [ ] **Step 4: Run the focused and existing prelude tests**

Run:

```bash
cargo test --no-default-features --features ros2 ros2::prelude::tests
```

Expected: all prelude tests pass.

- [ ] **Step 5: Commit the isolated helper**

Do not commit unrelated existing Daedalus changes. Stage only the helper/test hunk after reviewing the patch:

```bash
git diff -- src/ros2/prelude.rs
git add -p src/ros2/prelude.rs
git commit -m "fix(ros2): add inverse ROS rotation basis"
```

### Task 2: ROS Gimbal Command Conversion

**Files:**
- Modify: `/home/robomaster/bevy_robomaster_simulator/src/ros2/plugin.rs`
- Test: `/home/robomaster/bevy_robomaster_simulator/src/ros2/plugin.rs`

- [ ] **Step 1: Write failing axis-isolation tests**

Add a `#[cfg(test)]` module containing:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn published_ros_rotation(command_rotation: Quat) -> Quat {
        let published_bevy_rotation = command_rotation
            * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, PI / 2.0);
        let transformed = transform(Transform::from_rotation(published_bevy_rotation));
        Quat::from_xyzw(
            transformed.rotation.x as f32,
            transformed.rotation.y as f32,
            transformed.rotation.z as f32,
            transformed.rotation.w as f32,
        )
    }

    #[test]
    fn pitch_command_maps_to_ros_pitch_only() {
        let actual = published_ros_rotation(ros_gimbal_command_to_bevy_muzzle(0.0, -10.0));
        let expected = Quat::from_euler(EulerRot::ZYX, 0.0, -10.0_f32.to_radians(), 0.0);
        assert!(actual.dot(expected).abs() > 1.0 - 1e-6);
    }

    #[test]
    fn yaw_command_maps_to_ros_yaw_only() {
        let actual = published_ros_rotation(ros_gimbal_command_to_bevy_muzzle(20.0, 0.0));
        let expected = Quat::from_euler(EulerRot::ZYX, 20.0_f32.to_radians(), 0.0, 0.0);
        assert!(actual.dot(expected).abs() > 1.0 - 1e-6);
    }
}
```

- [ ] **Step 2: Run both tests and verify RED**

Run:

```bash
cargo test --no-default-features --features ros2 command_maps_to_ros
```

Expected: compilation fails because `ros_gimbal_command_to_bevy_muzzle` does not exist.

- [ ] **Step 3: Implement the pure command conversion**

Import `ros_rotation_to_bevy`, then add:

```rust
fn ros_gimbal_command_to_bevy_muzzle(yaw_deg: f32, pitch_deg: f32) -> Quat {
    let ros_rotation = Quat::from_euler(
        EulerRot::ZYX,
        yaw_deg.to_radians(),
        pitch_deg.to_radians(),
        0.0,
    );
    ros_rotation_to_bevy(ros_rotation)
        * Quat::from_euler(EulerRot::ZYX, 0.0, 0.0, -PI / 2.0)
}
```

The trailing `-90 degree` rotation is the inverse of the fixed `+90 degree` rotation used by `capture_rune` when publishing `gimbal_world_rotation`.

- [ ] **Step 4: Use the helper in `process_subscription`**

Replace the direct `EulerRot::YXZ` construction with:

```rust
let yaw_deg = cmd.yaw as f32;
let pitch_deg = cmd.pitch as f32 - 90.0;
let yaw_f32 = yaw_deg.to_radians();
let pitch_f32 = pitch_deg.to_radians();
gimbal_data.local_yaw = yaw_f32;
gimbal_data.pitch = pitch_f32;
let expected_rotation = ros_gimbal_command_to_bevy_muzzle(yaw_deg, pitch_deg);
let current_rotation = muzzle_offset.0.rotation();
let delta = expected_rotation * current_rotation.inverse();
gimbal_transform.rotation = delta * gimbal_transform.rotation;
```

- [ ] **Step 5: Run focused tests and verify GREEN**

Run:

```bash
cargo test --no-default-features --features ros2 command_maps_to_ros
cargo test --no-default-features --features ros2 ros2::prelude::tests
```

Expected: all selected tests pass.

- [ ] **Step 6: Commit only the command conversion hunks**

```bash
git diff -- src/ros2/plugin.rs
git add -p src/ros2/plugin.rs
git commit -m "fix(ros2): align gimbal commands with ROS axes"
```

### Task 3: Build and Dynamic ROS2 Verification

**Files:**
- Verify: `/home/robomaster/bevy_robomaster_simulator/src/ros2/prelude.rs`
- Verify: `/home/robomaster/bevy_robomaster_simulator/src/ros2/plugin.rs`
- Verify: `/home/robomaster/rm_vision_2025/io/ros2/sim_gimbal.cpp`

- [ ] **Step 1: Run Daedalus formatting and complete ROS2 tests**

Run:

```bash
cargo fmt --check
source /opt/ros/humble/setup.bash
source install/setup.bash
cargo test --no-default-features --features ros2
```

Expected: formatting and all ROS2-feature tests pass.

- [ ] **Step 2: Build the release binary**

Run:

```bash
cargo build --release --no-default-features --features ros2
```

Expected: release build completes without errors.

- [ ] **Step 3: Start Daedalus with its runtime paths**

Run:

```bash
export BEVY_ASSET_ROOT="$HOME/bevy_robomaster_simulator"
export LD_LIBRARY_PATH="$HOME/bevy_robomaster_simulator/target/release/deps:$HOME/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/lib/rustlib/x86_64-unknown-linux-gnu/lib:$LD_LIBRARY_PATH"
./target/release/daedalus
```

Press F5 in the Daedalus window and verify the overlay shows `AutoAim=ON`.

- [ ] **Step 4: Verify pitch axis dynamically**

Publish `pitch: 80`, `90`, and `100` with `yaw: 0`, `distance: 0`, then sample `/gimbal_pose`. Expected: ROS yaw remains within 0.5 degrees while ROS pitch changes approximately `-10`, `0`, and `+10` degrees relative to the same vehicle orientation.

- [ ] **Step 5: Verify yaw axis dynamically**

Publish `yaw: -10`, `0`, and `10` with `pitch: 90`, then sample `/gimbal_pose`. Expected: ROS pitch remains within 0.5 degrees while ROS yaw changes approximately `-10`, `0`, and `+10` degrees relative to the same vehicle orientation.

- [ ] **Step 6: Re-run the C++ bridge tests**

Run:

```bash
cd /home/robomaster/rm_vision_2025
source /opt/ros/humble/setup.bash
source /home/robomaster/bevy_robomaster_simulator/install/setup.bash
cmake --build build --target sim_gimbal_pose_test sim_gimbal_command_test QYG_sentry_sim -j2
./build/sim_gimbal_pose_test
./build/sim_gimbal_command_test
```

Expected: both tests pass and `QYG_sentry_sim` builds.

- [ ] **Step 7: Inspect final scope**

Run `git status --short` in both repositories. Confirm no existing user modifications were reverted and only the intended Daedalus rotation hunks plus the approved plan artifacts were added.
