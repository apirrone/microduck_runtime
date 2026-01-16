# Training vs Runtime Consistency Check

This document verifies that the microduck_runtime implementation matches the RL training environment from mjlab_microduck.

## ✅ OBSERVATION ORDER - CORRECT

Both training and runtime use the same observation order:

### Policy Observations (51 dimensions total)

| Index | Name | Dimensions | Units | Training Source | Runtime Source |
|-------|------|------------|-------|-----------------|----------------|
| 0-2 | base_ang_vel | 3 | rad/s | IMU gyro sensor | imu.gyro |
| 3-5 | projected_gravity | 3 | unit vector | quat_apply_inverse(quat, [0,0,-1]) | normalized(-accel) |
| 6-8 | command | 3 | m/s, rad/s | velocity command [vx, vy, ωz] | command[3] |
| 9-22 | joint_pos | 14 | rad | actual_pos - default_pos | motor_state.positions - DEFAULT_POSITION |
| 23-36 | joint_vel | 14 | rad/s | joint velocities | motor_state.velocities |
| 37-50 | last_action | 14 | rad | previous action | last_action |

**Verification**: Observation order matches exactly between `observation.rs` and training config.

## ✅ ACTION ORDER - CORRECT

Both use the same 14 joint order:

| Index | Joint Name | Training | Runtime |
|-------|------------|----------|---------|
| 0 | left_hip_yaw | ✓ | ✓ |
| 1 | left_hip_roll | ✓ | ✓ |
| 2 | left_hip_pitch | ✓ | ✓ |
| 3 | left_knee | ✓ | ✓ |
| 4 | left_ankle | ✓ | ✓ |
| 5 | neck_pitch | ✓ | ✓ |
| 6 | head_pitch | ✓ | ✓ |
| 7 | head_yaw | ✓ | ✓ |
| 8 | head_roll | ✓ | ✓ |
| 9 | right_hip_yaw | ✓ | ✓ |
| 10 | right_hip_roll | ✓ | ✓ |
| 11 | right_hip_pitch | ✓ | ✓ |
| 12 | right_knee | ✓ | ✓ |
| 13 | right_ankle | ✓ | ✓ |

## ✅ ACTION PROCESSING - CORRECT

**Training**: `target_position = default_position + action`
- Action scale: 1.0
- Action offset: automatically set to default positions (use_default_offset=True)

**Runtime**: `motor_targets[i] = DEFAULT_POSITION[i] + action[i]`

**Verification**: Both add action offsets to default positions identically.

## ✅ FIXED: DEFAULT POSITIONS - NOW CORRECT

**Issue Found**: Ankle positions were mismatched by 0.1 rad (~6°)
**Status**: FIXED in commit

### Default Positions Comparison

| Joint | Training (HOME_FRAME) | Runtime (OLD) | Runtime (NEW) | Status |
|-------|----------------------|---------------|---------------|--------|
| left_hip_yaw | 0.0 | 0.0 | 0.0 | ✓ |
| left_hip_roll | 0.0 | 0.0 | 0.0 | ✓ |
| left_hip_pitch | 0.6 | 0.6 | 0.6 | ✓ |
| left_knee | -1.2 | -1.2 | -1.2 | ✓ |
| **left_ankle** | **0.6** | **0.5** ❌ | **0.6** ✅ | FIXED |
| neck_pitch | 0.0 | 0.0 | 0.0 | ✓ |
| head_pitch | 0.0 | 0.0 | 0.0 | ✓ |
| head_yaw | 0.0 | 0.0 | 0.0 | ✓ |
| head_roll | 0.0 | 0.0 | 0.0 | ✓ |
| right_hip_yaw | 0.0 | 0.0 | 0.0 | ✓ |
| right_hip_roll | 0.0 | 0.0 | 0.0 | ✓ |
| right_hip_pitch | -0.6 | -0.6 | -0.6 | ✓ |
| right_knee | 1.2 | 1.2 | 1.2 | ✓ |
| **right_ankle** | **-0.6** | **-0.7** ❌ | **-0.6** ✅ | FIXED |

## ✅ PROJECTED GRAVITY - CORRECT

**Training**: `quat_apply_inverse(root_quat, [0, 0, -1])`
- Gravity vector is NORMALIZED unit vector [0, 0, -1]
- Rotated into body frame using inverse quaternion

**Runtime**: `normalize(-accelerometer_reading)`
- Accelerometer measures gravity in body frame
- Negated and normalized to unit vector

**Note**: Due to uncalibrated accelerometer (Accel=0), a --pitch-offset workaround is used to compensate for the constant bias.

## ✅ UNITS - CORRECT

| Observation | Training Unit | Runtime Unit | Match |
|-------------|---------------|--------------|-------|
| base_ang_vel | rad/s | rad/s | ✓ |
| projected_gravity | unit vector (normalized) | unit vector (normalized) | ✓ |
| command | m/s, rad/s | m/s, rad/s | ✓ |
| joint_pos | rad (relative to default) | rad (relative to default) | ✓ |
| joint_vel | rad/s | rad/s | ✓ |
| action | rad (offset from default) | rad (offset from default) | ✓ |

## ✅ VELOCITY CONVERSION - VERIFIED

**Motor Velocity Conversion**: `0.229 RPM/count × (2π/60) = rad/s`

This matches the XL330 servo specification and can be verified using debug_motor_speed tool.

## ⚠️ KNOWN ISSUES

### 1. Uncalibrated Accelerometer
- **Issue**: BNO055 accelerometer doesn't calibrate (Accel=0)
- **Impact**: Constant offset in projected gravity (~-0.22 on X axis when flat)
- **Workaround**: Use `--pitch-offset` parameter to compensate
- **Proper Fix**: Investigate why BNO055 won't calibrate (may need movement during startup or hardware fix)

### 2. IMU Axis Remapping
- **Configuration**: Hardware axis remapping in BNO055 chip
  ```rust
  swap_x_with(AXIS_AS_Y)
  set_axis_sign(X_NEGATIVE | Y_NEGATIVE)
  ```
- **Result**: Robot frame = [-Sensor_Y, -Sensor_X, Sensor_Z]
- **Status**: Verified with test_imu3 and debug_imu tools

## ✅ SUMMARY

All critical parameters now match between training and runtime:

1. ✅ Observation order: CORRECT
2. ✅ Action order: CORRECT
3. ✅ Action processing: CORRECT
4. ✅ Default positions: FIXED (ankles corrected from 0.5→0.6 and -0.7→-0.6)
5. ✅ Projected gravity: CORRECT (with pitch offset workaround)
6. ✅ Units: CORRECT (all in rad, rad/s, and normalized unit vectors)
7. ✅ Velocity conversion: CORRECT

## Testing Recommendations

1. Test with `--pitch-offset 0.22` to compensate for accelerometer bias
2. Verify motor speeds match expected values using `debug_motor_speed`
3. Monitor policy observations with `debug_imu_observations --pitch-offset 0.22`
4. Start with dummy policy to verify no crashes, then try real policy

## References

- Training config: `~/MISC/mjlab_microduck/src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py`
- Robot constants: `~/MISC/mjlab_microduck/src/mjlab_microduck/robot/microduck_constants.py`
- Runtime observations: `src/observation.rs`
- Runtime motors: `src/motor.rs`
