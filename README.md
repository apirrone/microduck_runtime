# Microduck Runtime

Runtime for the Microduck robot. Runs on Raspberry Pi Zero 2W with 14 Dynamixel XL330 motors and a BNO055 IMU.

Main repo : https://github.com/apirrone/microduck

## Install command

```bash
curl -sSL https://raw.githubusercontent.com/apirrone/microduck_runtime/main/install.sh | bash
```

## RPI setup
- `scp rpi_setup/config.txt in /boot/firmware/`
- run `sudo raspi-config`
    - interface
        - enable i2c
        - disable serial console, enable serial port
-  `sudo reboot 0`
- `curl -sSL https://raw.githubusercontent.com/apirrone/microduck_runtime/main/install.sh | bash`
- (Optional if trouble connecting the bluetooth controller. I use a xbox one controller btw)
    - `sudo btmgmt power off`
    - `sudo btmgmt privacy on`
    - `sudo btmgmt power on`
- run `bluetoothctl`
    - scan on
    - pair <mac>
    - connect <mac>
    - trust <mac>
- run `test_controller`

## Run : 

```bash
microduck_runtime
```

## Controller

| Button | Action |
|--------|--------|
| Start | Enable / disable policy |
| X | Toggle head mode |
| B | Toggle body pose mode |
| A | Trigger ground pick |
| Left stick | Forward / strafe |
| Right stick Y | Turn |
| Right trigger | Open/close mouth |

**Head mode** (X button) — joysticks control head joints:

| Stick | Joint |
|-------|-------|
| Left stick X | Head pitch |
| Left stick Y | Head yaw |
| Right stick X | Neck pitch |
| Right stick Y | Head roll |

Head offsets are preserved when switching back to walking mode.

**Body pose mode** (B button) — joysticks control standing body pose:

| Stick | Action |
|-------|--------|
| Left stick X | Z height (±25 mm) |
| Right stick X | Pitch (±20°) |
| Right stick Y | Roll (±20°) |

**Fall detection**: if the robot is detected as fallen for 0.2s, the policy stops and motors go limp (kP=50). Press Start to recover.

## Commands

These commands are available after installing with the curl command : 

| Binary | Description |
|--------|-------------|
| `microduck_runtime` | Main runtime — runs the policy and controls the robot |
| `init` | Enable torque on all motors |
| `em` | Emergency stop — disable torque on all motors |
| `check_voltage` | Check power supply voltage |
| `calibrate_imu` | Calibrate the IMU and save calibration to file |
| `test_controller` | Test gamepad input with a visual bar display |
| `test_controller_raw` | Dump raw gilrs events from the gamepad (useful for debugging axis/button mappings) |
| `test_imu` | Verify BNO055 IMU is working |
| `test_imu2` | Low-level I2C IMU acceleration data reading test |
| `test_imu3` | BNO055 IMU test with hardware axis remapping |
| `test_i2c_raw` | Raw I2C communication test with the BNO055 |
