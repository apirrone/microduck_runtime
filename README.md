# Microduck Runtime

Runtime for the Microduck robot. Runs on Raspberry Pi Zero 2W with 15 Dynamixel XL330 motors and a BNO055 IMU.


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

## Maploc — autonomous mapping & navigation

The `--maploc` flag enables 2D ToF-based mapping, MCL localization, and A* path planning, provided by the [`microduck_maploc-rs`](https://github.com/apirrone/microduck_maploc_rs) crate. It runs autonomously: the duck builds an occupancy grid as it moves, localizes against it, and (when given a goal) plans + executes a path.

```bash
# Silent autonomous mode — duck maps + localizes; goals come from on-board logic.
microduck_runtime --maploc

# Same plus laptop visualization & click-to-goto.
microduck_runtime --maploc --stream
```

`--maploc` is independent of `--stream`. Pairing the two enables the maploc telemetry/goal sockets without affecting the existing 9870 digital-twin protocol.

| Port  | Direction          | Payload                                |
|-------|--------------------|----------------------------------------|
| 9874  | duck → laptop      | pose, scan, map blob, planned path     |
| 9875  | laptop → duck      | goal click `(x, y)` in world frame     |

Map persistence: `/var/lib/microduck/maploc_map.bin` (~26 KB), auto-loaded on start, saved on exit. `--maploc-wipe` starts fresh; `--maploc-map-path` overrides the location.

To view the live map and click goals from the laptop, use `view_remote.py` from [microduck_maploc](../microduck_maploc):

```bash
uv run --extra sim python -m microduck_maploc.sim.view_remote --host <duck-ip>
```

ToF hardware: a VL53L5CX driver is the missing piece. The `--maploc` path runs today against `tof::NoopTof` (no measurement updates — pose drifts on odometry alone). Once the sensor lands, the driver in `src/tof.rs` plugs in behind the existing `TofSource` trait.

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
