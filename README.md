# Microduck Runtime

Runtime for the Microduck robot. Runs on Raspberry Pi Zero 2W with 14 Dynamixel XL330 motors and a BNO055 IMU.

## Install

```bash
curl -sSL https://raw.githubusercontent.com/apirrone/microduck_runtime/main/install.sh | bash
```

## Recommended command

```bash
microduck_runtime -m output.onnx --action-scale 0.5 --head-max 1.5 --max-linear-vel 0.5 --max-angular-vel 4 --gravity-offset-x 0.1 -c
```

## Controller

| Button | Action |
|--------|--------|
| Start | Enable / disable policy |
| Y | Toggle head mode (joysticks control head joints instead of velocity) |
| Left stick | Forward / strafe |
| Right stick Y | Turn |

**Head mode** (Y button):

| Stick | Joint |
|-------|-------|
| Left stick X | Head pitch |
| Left stick Y | Head yaw |
| Right stick X | Neck pitch |
| Right stick Y | Head roll |

Head offsets are preserved when switching back to walking mode.

**Fall detection**: if the robot is detected as fallen for 0.2s, the policy stops and motors go limp (kP=50). Press Start to recover.

## Binaries

| Binary | Description |
|--------|-------------|
| `microduck_runtime` | Main runtime — runs the policy and controls the robot |
| `init` | Enable torque on all motors |
| `em` | Emergency stop — disable torque on all motors |
| `check_voltage` | Check power supply voltage |
| `calibrate_imu` | Calibrate the IMU and save calibration to file |
| `debug_imu` | Display IMU euler angles and quaternion data |
| `debug_imu_observations` | Display live IMU observations as seen by the policy |
| `debug_motor_speed` | Monitor motor speeds |
| `debug_policy_io` | Test policy input/output with live motors and IMU |
| `test_controller` | Test gamepad input with a visual bar display |
| `test_imu` | Verify BNO055 IMU is working |
| `test_imu_latency` | Measure IMU read latency |
| `test_imu_fusion_latency` | Measure IMU sensor fusion latency |
| `test_motor_latency` | Measure motor command latency |
| `test_i2c_raw` | Raw I2C communication test with the BNO055 |
