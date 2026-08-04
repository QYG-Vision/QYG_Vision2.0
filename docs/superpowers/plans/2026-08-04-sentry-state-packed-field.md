# Sentry State Packed Field Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Encode the GD receive-side sentry status and vision mode in one little-endian `uint16_t` while preserving the 51-byte frame layout.

**Architecture:** `ReceiveFrame` and `GimbalState` store the wire value as `sentry_state`. Protocol helpers encode/decode the low two mode bits and the upper fourteen state bits. `Gimbal`, `Aim2Nav`, and the raw-debug tool consume the helpers instead of independent numeric mappings.

**Tech Stack:** C++17, packed serial protocol structs, CMake, ROS 2, existing `protocol_ros_loop_test` executable.

## Global Constraints

- GD `ReceiveFrame` remains exactly 51 bytes.
- `sentry_state` occupies former bytes 36-37 and is little-endian on the wire.
- Bits 1:0 are mode: `00` idle, `01` auto aim, `10` small buff, `11` big buff.
- Bits 15:2 are a 14-bit sentry status value.
- Do not use C++ bit-fields for wire data.

---

### Task 1: Define and test packed-state helpers

**Files:**
- Modify: `io/gimbal/gimbal.hpp`
- Modify: `io/gimbal/gimbal_protocol.hpp`
- Modify: `io/gimbal/gimbal_protocol.cpp`
- Test: `tests/protocol_ros_loop_test.cpp`

**Interfaces:**
- Produces: `uint16_t pack_sentry_state(uint16_t status, GimbalMode mode)`
- Produces: `uint16_t sentry_status(uint16_t sentry_state)`
- Produces: `GimbalMode sentry_mode(uint16_t sentry_state)`

- [ ] **Step 1: Write the failing test**

```cpp
expect_equal_hex(pack_sentry_state(0x1234, GimbalMode::BIG_BUFF), 0x48D3, ...);
expect_equal_hex(sentry_status(0x48D3), 0x1234, ...);
expect_true(sentry_mode(0x48D3) == GimbalMode::BIG_BUFF, ...);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build /tmp/qyg_sentry_state_baseline --target protocol_ros_loop_test -j$(nproc)`

Expected: compilation fails because the packed-state helper interface does not exist.

- [ ] **Step 3: Write minimal implementation**

```cpp
constexpr uint16_t kModeMask = 0x0003;
constexpr uint16_t kStatusMask = 0x3FFF;
return static_cast<uint16_t>((status & kStatusMask) << 2) |
       static_cast<uint16_t>(mode);
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build /tmp/qyg_sentry_state_baseline --target protocol_ros_loop_test -j$(nproc) && /tmp/qyg_sentry_state_baseline/protocol_ros_loop_test`

Expected: packed-state helper checks and existing tests pass.

### Task 2: Replace the two frame fields and update consumers

**Files:**
- Modify: `io/gimbal/gimbal.hpp`
- Modify: `io/gimbal/gimbal.cpp`
- Modify: `io/gimbal/gimbal_protocol.cpp`
- Modify: `io/ros2/aim2nav.hpp`
- Modify: `io/ros2/aim2nav.cpp`
- Modify: `tests/gimbal_raw_debug.cpp`
- Test: `tests/protocol_ros_loop_test.cpp`

**Interfaces:**
- Consumes: packed-state helpers from Task 1.
- Produces: `Gimbal::mode()` and `Aim2Nav::get_mode()` based on `sentry_state`.

- [ ] **Step 1: Write the failing parsing/layout test**

```cpp
rx.sentry_state = pack_sentry_state(0x1234, GimbalMode::AUTO_AIM);
expect_equal_hex(parsed_state->sentry_state, 0x48D1, ...);
expect_true(bytes[35] == 0xD1 && bytes[36] == 0x48, ...);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build /tmp/qyg_sentry_state_baseline --target protocol_ros_loop_test -j$(nproc)`

Expected: compilation fails because `ReceiveFrame::sentry_state` is absent.

- [ ] **Step 3: Write minimal implementation**

```cpp
uint16_t sentry_state = 0;
mode_ = gimbal_protocol::sentry_mode(rx.sentry_state);
```

Keep the existing size and offset assertions, changing the first relevant
assertion to `offsetof(ReceiveFrame, sentry_state) == 35`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build /tmp/qyg_sentry_state_baseline --target protocol_ros_loop_test QYG_sentry_debug gimbal_raw_debug -j$(nproc) && /tmp/qyg_sentry_state_baseline/protocol_ros_loop_test`

Expected: all tests pass and both serial debug executables link.

### Task 3: Publish the protocol agreement

**Files:**
- Create: `docs/superpowers/specs/2026-08-04-sentry-state-design.md`

- [ ] **Step 1: Review the documentation**

Run: `rg -n "sentry_state|little-endian|0b00|0b01|0b10|0b11|51 bytes" docs/superpowers/specs/2026-08-04-sentry-state-design.md`

Expected: the on-wire layout, all four modes, and migration constraint are explicit.

- [ ] **Step 2: Commit only scoped files and push**

```bash
git add docs/superpowers/specs/2026-08-04-sentry-state-design.md \
  docs/superpowers/plans/2026-08-04-sentry-state-packed-field.md \
  io/gimbal/gimbal.hpp io/gimbal/gimbal.cpp io/gimbal/gimbal_protocol.hpp \
  io/gimbal/gimbal_protocol.cpp io/ros2/aim2nav.hpp io/ros2/aim2nav.cpp \
  tests/gimbal_raw_debug.cpp tests/protocol_ros_loop_test.cpp
git commit -m "Pack sentry state and vision mode"
git push -u origin qlf-clean
```
