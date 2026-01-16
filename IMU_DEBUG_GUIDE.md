# IMU Debugging Guide

This guide explains how to verify that your BNO055 IMU is providing data in the correct orientation and units for the microduck robot.

## Expected Frame Convention

The microduck runtime expects the robot frame to be:
- **X = forward** (direction the robot walks)
- **Y = left** (perpendicular to forward, pointing left)
- **Z = up** (perpendicular to ground, pointing up)

Rotations are defined as:
- **Roll**: rotation around X axis (forward) - tilting left/right
- **Pitch**: rotation around Y axis (left) - tilting forward/back
- **Yaw**: rotation around Z axis (up) - rotating horizontally

## BNO055 Frame Convention and Hardware Remapping

The BNO055 uses Android coordinate system:
- **X = right** (when looking at chip)
- **Y = forward** (when looking at chip)
- **Z = up** (out of chip surface)

**Hardware axis remapping (configured in BNO055 chip):**

The IMU is configured using the bno055 crate's hardware axis remapping:
```rust
AxisRemap::builder()
    .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)  // Swap X and Y
    .build()

imu.set_axis_sign(BNO055AxisSign::Y_NEGATIVE)  // Flip Y sign
```

This results in the robot frame (X=forward, Y=left, Z=up):
- **Robot X** = Sensor Y (forward)
- **Robot Y** = -Sensor X (left, negated from right)
- **Robot Z** = Sensor Z (up)

All sensor readings (accelerometer, gyroscope, quaternions) are automatically
transformed by the hardware. No software transformation is needed.

## Using the Debug Tool

### 1. Run the debug tool

```bash
sudo debug_imu
```

This will:
1. Initialize the BNO055
2. Wait for calibration (move the robot around for ~10 seconds)
3. Display real-time IMU data with:
   - Gyroscope (angular velocity in rad/s)
   - Quaternion (normalized, should have magnitude ~1.0)
   - Euler angles (roll, pitch, yaw in degrees)
   - Projected gravity (gravity vector in body frame)

### 2. Verify Calibration

The tool shows calibration status: `System | Gyro | Accel | Mag`

Each value ranges from 0 (uncalibrated) to 3 (fully calibrated). For testing:
- Gyro should be at least 2
- System should be at least 2

### 3. Test Orientation

With the robot on a flat surface:

#### Test 1: Upright Position
- **Projected Gravity should be**: [0, 0, -9.81] (approximately)
  - X ≈ 0, Y ≈ 0, Z ≈ -9.81
- **Euler angles should be**: ~0° for all
- **Gyro should be**: ~0 rad/s for all axes (when stationary)

#### Test 2: Pitch (Tilt Forward/Backward)
- Tilt robot forward (nose down)
- **Pitch should increase** (positive angle)
- **Projected Gravity X should become NEGATIVE** (e.g., -0.4 at 22° pitch)
- This matches MuJoCo behavior: gravity appears to point backward when nose is down

- Tilt robot backward (nose up)
- **Pitch should decrease** (negative angle)
- **Projected Gravity X should become positive** (gravity appears to point forward when nose is up)

#### Test 3: Roll (Tilt Left/Right)
- Tilt robot to the right (right side down)
- **Roll should increase** (positive angle)
- **Projected Gravity Y should become POSITIVE** (gravity appears to point left when tilted right)
- This matches MuJoCo behavior

- Tilt robot to the left (left side down)
- **Roll should decrease** (negative angle)
- **Projected Gravity Y should become negative** (gravity appears to point right when tilted left)

#### Test 4: Yaw (Rotate Horizontally)
- Rotate robot clockwise (viewed from above)
- **Yaw should change**
- **Gyro Z should show positive rotation** (right-hand rule around Z axis)

### 4. Verify Units

- **Gyroscope**: Should be in rad/s
  - Slow rotation (~1 rev/sec) should show ~6.28 rad/s (2π)
  - Fast rotation (~2 rev/sec) should show ~12.56 rad/s (4π)

- **Quaternion**: Should be normalized
  - w² + x² + y² + z² ≈ 1.0
  - Tool shows quaternion magnitude - should be ~1.0

- **Projected Gravity**: Should be ~9.81 m/s² magnitude
  - √(X² + Y² + Z²) ≈ 9.81
  - Tool warns if magnitude is off by more than 0.5 m/s²

## Common Issues

### Issue: Axes are swapped
**Symptom**: Tilting forward changes roll instead of pitch, etc.
**Cause**: BNO055 is mounted in a different orientation than expected
**Solution**: Need to transform IMU data by swapping/negating axes in `src/imu.rs`

### Issue: Angles are inverted
**Symptom**: Tilting forward gives negative pitch instead of positive
**Cause**: Axis direction is flipped
**Solution**: Negate the corresponding quaternion component or projected gravity axis

### Issue: Quaternion magnitude is not 1.0
**Symptom**: Quaternion magnitude shows values far from 1.0 (e.g., 0.5 or 2.0)
**Cause**: Scale factor is wrong in quaternion conversion
**Solution**: Verify BNO055 datasheet for correct scale (should be 1/16384)

### Issue: Projected gravity magnitude is not 9.81
**Symptom**: Gravity magnitude shows values far from 9.81 m/s²
**Cause**: Quaternion rotation is incorrect, or sensor is picking up acceleration
**Solution**:
1. Keep robot stationary (no linear acceleration)
2. Verify quaternion is correct (magnitude = 1.0)
3. Check quaternion rotation formula

### Issue: Gyro shows large values when stationary
**Symptom**: Gyroscope readings are not ~0 when robot is still
**Cause**:
1. Sensor not calibrated (gyro calibration < 2)
2. Temperature drift
3. Vibrations
**Solution**:
1. Wait for calibration to complete
2. Keep sensor still during startup
3. Ensure robot is on stable surface

## Expected Output Example

When robot is upright and stationary:
```
┌─────────────────────────────────────────────────────────────┐
│ Gyro (rad/s):      X= -0.001  Y=  0.002  Z=  0.000      │
│ Quaternion [WXYZ]:  1.000 -0.001  0.002  0.001 (mag=1.000) │
│ Euler (deg):       Roll=   0.10° Pitch=   0.20° Yaw=   0.15° │
│ ProjGrav (m/s²):   X=  0.034  Y= -0.012  Z= -9.809      │
└─────────────────────────────────────────────────────────────┘
```

When tilted forward by ~30°:
```
┌─────────────────────────────────────────────────────────────┐
│ Gyro (rad/s):      X=  0.000  Y=  0.001  Z=  0.000      │
│ Quaternion [WXYZ]:  0.966  0.001  0.259  0.000 (mag=1.000) │
│ Euler (deg):       Roll=   0.00° Pitch=  30.00° Yaw=   0.00° │
│ ProjGrav (m/s²):   X= -4.905  Y=  0.000  Z= -8.495      │
└─────────────────────────────────────────────────────────────┘
```

## Next Steps

Once you've verified the IMU orientation:

1. If axes are correct: Great! The policy should work correctly.

2. If axes are swapped/inverted: You need to modify `src/imu.rs` to transform the IMU data:
   - Option A: Swap quaternion components (e.g., swap X and Y)
   - Option B: Add axis transformation after reading data
   - Option C: Physically remount the BNO055 in the correct orientation

3. If units are wrong: Verify the scale factors in `src/imu.rs` lines 169 and 181.
