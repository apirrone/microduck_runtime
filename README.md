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

## Xbox controller

### Normal setup :
- `bluetoothctl` : 
    - scan on (and long press pairing button on the controller)
    - connect <mac address>
    - wait for it to prompt you to accept pairing -> yes
    - trust <mac address>
    
- run `test_controller`

### If issues : 
    
I had this issue : when rebooting, the controller would be stuck in a connect/disconnect loop.

I would have to remove and repair the controller at each reboot.

The solution was to set Privacy to "on" in `/etc/bluetooth/main.conf`. 

Then reboot, remove the device (remove <mac> in bluetoothctl) and do the normal setup above.
       

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
| `init` | Enable torque on all motors and move to default pose |
| `em` | Emergency stop — disable torque on all motors |
| `check_voltage` | Read present voltage for all motors |
| `calibrate_imu` | Interactive IMU calibration wizard, saves to `~/.config/microduck/imu_calibration.bin` |
| `test_imu` | Stream IMU readings (accel, gyro) with axis remapping |
| `test_i2c_raw` | Raw I2C connectivity check for the BNO055 |
| `test_controller` | Display Xbox controller state with live bar graphs |
| `microduck_help` | List all available commands |
