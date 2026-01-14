# Microduck Robot Runtime

A performant Rust runtime for the Microduck robot, designed to run on Raspberry Pi Zero 2W with Dynamixel XL330 motors.

## Features

- **Motor Control**: Interfaces with 14 Dynamixel XL330 motors using [rustypot](https://github.com/pollen-robotics/rustypot)
- **IMU Support**: Placeholder for BNO055 IMU integration (dummy implementation for now)
- **Observation Space**: Collects 51-dimensional observation vectors for RL policy
- **ONNX Runtime**: Ready for policy inference using ONNX models (placeholder implementation)
- **Performance Optimized**: Designed for real-time control on Raspberry Pi Zero 2W
- **Clean Architecture**: Modular design with separate concerns

## Architecture

The runtime is organized into modules:

- `motor.rs` - XL330 motor controller with sync read/write operations
- `imu.rs` - IMU interface (dummy for BNO055, ready for real implementation)
- `observation.rs` - Observation space collector (51-dim vector)
- `policy.rs` - ONNX runtime policy inference (placeholder)
- `main.rs` - Main control loop with timing and error handling

## Observation Space (51 dimensions)

The observation vector contains:
- Gyroscope [x, y, z] (3)
- Accelerometer [x, y, z] (3)
- Command [x, y, z] (3)
- Joint positions **relative to default position** (14) - `dof_pos - init_pos`
- Joint velocities (14) - `dof_vel`
- Last action (14)

## Action Space (14 dimensions)

The policy outputs **offsets from the default position** (14 values).

The final motor targets are computed as:
```rust
motor_targets = DEFAULT_POSITION + action
```

This matches the Python implementation:
```python
self.motor_targets = self.init_pos + action
```

## Default Position

The default position (`DEFAULT_POSITION` in `src/motor.rs`) represents the robot's initial stance with bent legs.

Currently set to all zeros:
```rust
pub const DEFAULT_POSITION: [f64; NUM_MOTORS] = [0.0; NUM_MOTORS];
```

**TODO**: Update these values to match your robot's actual default stance. The order follows the motor ordering above (left leg, neck/head, right leg):

```rust
pub const DEFAULT_POSITION: [f64; NUM_MOTORS] = [
    // Left leg (20-24)
    0.0, 0.0, 0.0, 0.0, 0.0,
    // Neck/head (30-33)
    0.0, 0.0, 0.0, 0.0,
    // Right leg (10-14)
    0.0, 0.0, 0.0, 0.0, 0.0,
];
```

## Configuration

Configuration is done via command-line arguments:

```bash
# View all options
cargo run --release -- --help

# Basic options
--port <PORT>          Serial port for motor communication [default: /dev/ttyUSB0]
--baudrate <BAUDRATE>  Motor communication baudrate [default: 1000000]
--freq <FREQ>          Control loop frequency in Hz [default: 100]
--dummy                Use dummy policy (always outputs zeros) for testing
--model <MODEL>        Path to ONNX model file
```

**Motor IDs and Ordering**:

The 14 motors are organized as: left leg (5), neck/head (4), right leg (5)

| Index | Motor ID | Joint Name       | Body Part |
|-------|----------|------------------|-----------|
| 0     | 20       | left_hip_yaw     | Left Leg  |
| 1     | 21       | left_hip_roll    | Left Leg  |
| 2     | 22       | left_hip_pitch   | Left Leg  |
| 3     | 23       | left_knee        | Left Leg  |
| 4     | 24       | left_ankle       | Left Leg  |
| 5     | 30       | neck_pitch       | Neck/Head |
| 6     | 31       | head_pitch       | Neck/Head |
| 7     | 32       | head_yaw         | Neck/Head |
| 8     | 33       | head_roll        | Neck/Head |
| 9     | 10       | right_hip_yaw    | Right Leg |
| 10    | 11       | right_hip_roll   | Right Leg |
| 11    | 12       | right_hip_pitch  | Right Leg |
| 12    | 13       | right_knee       | Right Leg |
| 13    | 14       | right_ankle      | Right Leg |

This ordering is used consistently in observation and action vectors.

## Building

```bash
# Development build
cargo build

# Release build (optimized for Raspberry Pi)
cargo build --release
```

## Running

```bash
# Make sure the motors are connected via USB serial adapter
sudo chmod 666 /dev/ttyUSB0  # Grant permissions (or run as root)

# Run with dummy policy (outputs zeros - motors stay at default position)
cargo run --release -- --dummy

# Run with custom port and frequency
cargo run --release -- --dummy --port /dev/ttyACM0 --freq 50

# Run with ONNX model (when you have one)
cargo run --release -- --model /path/to/model.onnx

# Run with ONNX model and custom settings
cargo run --release -- --model policy.onnx --freq 200 --baudrate 1000000
```

The runtime will:
1. Initialize motor controller
2. Initialize IMU (dummy mode)
3. Load policy (dummy or ONNX)
4. Enable torque on all motors
5. Start the control loop at specified Hz
6. Print statistics every second

Press Ctrl+C to safely shutdown. The runtime will:
- Detect the Ctrl+C signal
- Exit the control loop cleanly
- Close all connections
- **Motors remain enabled** at their last commanded position

The shutdown should take less than 1 second.

**Note**: Motors will NOT be disabled on exit - they will hold their last position. To disable motors, you need to either:
- Power off the motors
- Run a separate command to disable torque
- Modify the `shutdown()` method to call `set_torque_enable(false)`

### Testing Before Training

Use the `--dummy` flag to test your hardware setup without a trained policy. The dummy policy always outputs zero offsets, so motors will maintain their default positions.

## Adding ONNX Policy

To integrate your trained RL policy:

1. Uncomment the ONNX code in `src/policy.rs` (in the `new_onnx` and `infer` methods)
2. Place your `.onnx` model file on the Raspberry Pi
3. Run with the `--model` flag:

```bash
cargo run --release -- --model /path/to/your/model.onnx
```

The policy will receive 51-dimensional observations and output 14-dimensional action offsets.

## Implementing Real IMU

When you connect the BNO055 IMU, update `src/imu.rs`:

1. Add BNO055 driver crate (e.g., `bno055` from crates.io)
2. Replace the dummy implementation with real I2C communication
3. Convert sensor data to appropriate units (rad/s for gyro, m/s² for accel)

Example:
```rust
// In src/imu.rs
use bno055::{Bno055, BNO055_DEFAULT_ADDR};

pub struct ImuController {
    sensor: Bno055<I2c>,
}

impl ImuController {
    pub fn new() -> Result<Self> {
        // Initialize I2C and BNO055
        let i2c = I2c::new(...)?;
        let sensor = Bno055::new(i2c)?;
        Ok(Self { sensor })
    }

    pub fn read(&mut self) -> Result<ImuData> {
        let gyro = self.sensor.gyro()?;
        let accel = self.sensor.accel()?;
        Ok(ImuData {
            gyro: [gyro.x, gyro.y, gyro.z],
            accel: [accel.x, accel.y, accel.z],
        })
    }
}
```

## Performance Considerations

For Raspberry Pi Zero 2W:
- Using sync read/write for efficient communication with multiple motors
- 100 Hz control loop (10ms per iteration)
- Release build with optimizations enabled
- ONNX Runtime configured with limited threads

## Conversion Functions

Motor velocity conversion from XL330 raw values:
```rust
velocity_rad_per_sec = raw_value * 0.229 * (2π / 60)
```

This matches the Python implementation from `~/Rhoban/bam/bam/xl330/record.py`.

## Dependencies

- `rustypot` - Motor communication
- `serialport` - Serial port access
- `ort` - ONNX Runtime bindings
- `anyhow` - Error handling
- `ctrlc` - Graceful shutdown

## License

Same as your project.

## Example Workflow

1. **Initial Hardware Test** - Test motors with dummy policy:
   ```bash
   cargo run --release -- --dummy
   ```
   Motors should initialize at default position and stay there.

2. **Adjust Default Position** - Update `DEFAULT_POSITION` in `src/motor.rs` to match your robot's stance.

3. **Test at Different Frequencies**:
   ```bash
   cargo run --release -- --dummy --freq 50   # Lower freq for testing
   cargo run --release -- --dummy --freq 200  # Higher freq for performance
   ```

4. **Deploy Trained Policy** - Once you have an ONNX model:
   ```bash
   cargo run --release -- --model microduck_policy.onnx
   ```

5. **Integrate Real IMU** - Implement BNO055 driver in `src/imu.rs`

## Next Steps

1. Test with actual hardware using `--dummy` flag
2. Calibrate and set `DEFAULT_POSITION` values
3. Train RL policy and export to ONNX
4. Integrate trained ONNX policy
5. Connect and implement BNO055 IMU
6. Tune control loop frequency based on performance
7. Add telemetry/logging for debugging
