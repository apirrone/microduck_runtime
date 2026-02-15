# Gyroscope Spike Filtering

## Problem

The BNO055 IMU occasionally produces large spikes in gyroscope readings (e.g., 2.2 rad/s in ω_x while other axes remain stable). These spikes are likely caused by:

1. **I2C communication errors**: Transient bit errors during sensor communication
2. **Sensor glitches**: The BNO055 chip occasionally producing spurious readings
3. **Electromagnetic interference**: Motors, PWM signals, or power electronics creating noise

## Solution: Latency-Free Outlier Detection

A simple outlier detection system has been implemented that:

1. **Validates angular acceleration**: Checks if gyro change between samples exceeds physical limits
2. **Rejects outliers**: If a spike is detected, uses the previous valid reading instead
3. **Zero latency**: No filtering or averaging that would introduce phase lag
4. **Minimal computation**: Just 3 comparisons per IMU reading

## Implementation Details

### Algorithm

For each gyroscope reading:
```
max_delta = max_angular_accel × dt
for each axis (x, y, z):
    if |gyro_new[i] - gyro_prev[i]| > max_delta:
        reject reading and use gyro_prev
```

### Default Parameters

- **Max angular acceleration**: 100 rad/s² (configurable)
- **Control frequency**: 50 Hz (dt = 0.02s)
- **Max change per step**: 100 × 0.02 = 2.0 rad/s

This means any reading that changes by more than 2.0 rad/s in a single step (20ms) will be rejected.

## Usage

### Default Behavior (Enabled)

By default, outlier detection is **enabled** with a threshold of 100 rad/s²:

```bash
sudo microduck_runtime --model policy.onnx
```

### Adjust Threshold

To change the maximum angular acceleration threshold:

```bash
# More aggressive (reject smaller spikes)
sudo microduck_runtime --model policy.onnx --max-angular-accel 50

# More lenient (only reject very large spikes)
sudo microduck_runtime --model policy.onnx --max-angular-accel 200
```

### Disable Outlier Detection

To disable outlier detection completely (not recommended):

```bash
sudo microduck_runtime --model policy.onnx --no-gyro-outlier-filter
```

## Statistics

At shutdown, the system will report how many samples were rejected:

```
⚠  Gyro outliers rejected: 47 samples (0.15% of 30000 total)
```

If you see a high rejection rate (>1%), it may indicate:
- **I2C communication issues**: Check wiring and pull-up resistors
- **Electromagnetic interference**: Check shielding and grounding
- **Threshold too strict**: Increase `--max-angular-accel` value

## Tuning Guidance

### For Different Control Frequencies

If you change the control frequency, you may need to adjust the threshold:

- **25 Hz** (dt=0.04s): Use `--max-angular-accel 50` (50 × 0.04 = 2.0 rad/s max change)
- **50 Hz** (dt=0.02s): Use `--max-angular-accel 100` (default)
- **100 Hz** (dt=0.01s): Use `--max-angular-accel 200` (200 × 0.01 = 2.0 rad/s max change)

The key is keeping the **max change per step** around 2.0 rad/s, which is a reasonable limit for a quadruped robot.

### For Different Robot Dynamics

- **Slow, stable gaits**: Lower threshold (50-75 rad/s²)
- **Fast, dynamic gaits**: Higher threshold (100-150 rad/s²)
- **Extremely dynamic motions**: Consider disabling (but monitor for spikes)

## Technical Details

### Code Changes

1. **src/imu.rs**: Added outlier detection logic in `ImuController::read()`
   - Lines 29-37: Added fields to track previous gyro and rejected samples
   - Lines 283-306: Outlier detection algorithm

2. **src/main.rs**: Added command-line arguments
   - `--max-angular-accel`: Set threshold (default: 100 rad/s²)
   - `--no-gyro-outlier-filter`: Disable outlier detection

### Performance Impact

- **Computation**: ~3 floating-point comparisons per reading (negligible)
- **Memory**: 3 × 8 bytes = 24 bytes additional storage
- **Latency**: Zero (no filtering or buffering)

## Verification

To verify the outlier detection is working:

1. Run with logging enabled:
```bash
sudo microduck_runtime --model policy.onnx --log-file imu_data.csv
```

2. After running, check the rejection statistics at shutdown

3. Plot the gyro data from the CSV file - you should see no large spikes

4. If you want to see the raw data without filtering:
```bash
sudo microduck_runtime --model policy.onnx --no-gyro-outlier-filter --log-file imu_raw.csv
```

Compare the two datasets to see which spikes were rejected.
