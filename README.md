# Microduck Runtime

Runtime for the Microduck robot. Runs on Raspberry Pi Zero 2W with 14 Dynamixel XL330 motors and a BNO055 IMU.

Main repo : https://github.com/apirrone/microduck

## Install command

Run this command everytime you want to update the runtime. If it's the first time setting up, follow RPI setup below

```bash
curl -sSL https://raw.githubusercontent.com/apirrone/microduck_runtime/main/install.sh | bash
```

## RPI setup
Docs : [here](rpi_setup/setup.md)


## Usage : 

After installing everything, a systemd service is running at boot which runs everything automatically. 

You turn on the robot, turn on your xbox controller and wait for all the motors of the robot to blink with a heartbeat pattern.

Then press start to go to init mode, and start again to start infering the policies.

If you are experimenting with policies, you may want to run stuff yourself. You can disable the service with : 

```bash
sudo systemctl disable microduck_runtime.service
```

And then you can run the runtime binary yourself with :

```bash
microduck_runtime
```

Use `-h` to see the options

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
| `microduck_help` | List all available commands |
| `microduck_runtime` | Main runtime — runs the policy and controls the robot |
| `init` | Enable torque on all motors and move to default pose |
| `em` | Emergency stop — disable torque on all motors |
| `check_voltage` | Read present voltage for all motors |
| `calibrate_imu` | Interactive IMU calibration wizard, saves to `~/.config/microduck/imu_calibration.bin` |
| `test_imu` | Stream IMU readings (accel, gyro) with axis remapping |
| `test_i2c_raw` | Raw I2C connectivity check for the BNO055 |
| `test_controller` | Display Xbox controller state with live bar graphs |
