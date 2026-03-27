use anyhow::{Context, Result};
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::f64::consts::PI;
use std::time::Duration;

/// Feature flag: use bulk read (faster but may be less reliable)
/// Set to false to use separate position/velocity reads (slower but more reliable)
const USE_BULK_READ: bool = true;

/// Number of motors on the microduck robot
pub const NUM_MOTORS: usize = 14;

/// Mouth motor ID (independent from policy)
pub const MOUTH_MOTOR_ID: u8 = 34;

/// Mouth motor range: -10° to 70°
pub const MOUTH_MIN_ANGLE: f64 = -10.0 * PI / 180.0;
pub const MOUTH_MAX_ANGLE: f64 = 70.0 * PI / 180.0;

/// Motor IDs in order: left leg (5), neck/head (4), right leg (5)
/// This order matches the observation/action vector indices
pub const MOTOR_IDS: [u8; NUM_MOTORS] = [
    20, // [0]  left_hip_yaw
    21, // [1]  left_hip_roll
    22, // [2]  left_hip_pitch
    23, // [3]  left_knee
    24, // [4]  left_ankle
    30, // [5]  neck_pitch
    31, // [6]  head_pitch
    32, // [7]  head_yaw
    33, // [8]  head_roll
    10, // [9]  right_hip_yaw
    11, // [10] right_hip_roll
    12, // [11] right_hip_pitch
    13, // [12] right_knee
    14, // [13] right_ankle
];

/// Default/initial position for all motors (bent legs position)
/// Order matches MOTOR_IDS: left leg (5), neck/head (4), right leg (5)
/// These values MUST match HOME_FRAME in microduck_constants.py for consistency with RL training
pub const DEFAULT_POSITION: [f64; NUM_MOTORS] = [
    0.0, // left_hip_yaw
    0.0, // left_hip_roll
    0.6, // left_hip_pitch
    -1.2, // left_knee
    0.6, // left_ankle (was 0.5 - FIXED to match training)

    0.0, // neck_pitch
    0.0, // head_pitch
    0.0, // head_yaw
    0.0, // head_roll

    0.0, // right_hip_yaw
    0.0, // right_hip_roll
    -0.6, // right_hip_pitch
    1.2, // right_knee
    -0.6, // right_ankle (was -0.7 - FIXED to match training)
];

/// Conversion factor for velocity: 0.229 RPM per count * (2π / 60) for rad/s
const RADS_PER_SEC_PER_COUNT: f64 = 0.229 * (2.0 * PI / 60.0);

/// Motor state containing position, velocity and current
#[derive(Debug, Clone)]
pub struct MotorState {
    pub positions: [f64; NUM_MOTORS],
    pub velocities: [f64; NUM_MOTORS],
    /// Absolute present current for each motor in mA
    pub currents_ma: [f64; NUM_MOTORS],
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            positions: [0.0; NUM_MOTORS],
            velocities: [0.0; NUM_MOTORS],
            currents_ma: [0.0; NUM_MOTORS],
        }
    }
}

/// Motor controller for XL330 servos
pub struct MotorController {
    controller: Xl330Controller,
}

impl MotorController {
    /// Create a new motor controller
    pub fn new(port: &str, baudrate: u32) -> Result<Self> {
        // Open serial port with increased timeout for bulk reads
        // Reading 8 bytes from 14 motors requires waiting for 14 status packets
        // Increased to 500ms to rule out timeout issues
        let serial_port = serialport::new(port, baudrate)
            .timeout(Duration::from_millis(500))
            .open()
            .context("Failed to open serial port")?;

        // Create controller with XL330 (uses Protocol v2)
        let controller = Xl330Controller::new()
            .with_protocol_v2()
            .with_serial_port(serial_port);

        Ok(Self { controller })
    }

    /// Read current state of all motors (position and velocity)
    /// Automatically selects bulk read or separate reads based on USE_BULK_READ flag
    pub fn read_state(&mut self) -> Result<MotorState> {
        if USE_BULK_READ {
            self.read_state_bulk()
        } else {
            self.read_state_separate()
        }
    }

    /// Read current state using single bulk sync_read (faster)
    /// Reads 10 consecutive bytes starting at address 126:
    ///   bytes 0-1: present current  (i16, mA)
    ///   bytes 2-5: present velocity (i32)
    ///   bytes 6-9: present position (i32)
    fn read_state_bulk(&mut self) -> Result<MotorState> {
        let mut state = MotorState::default();

        const CURRENT_ADDR: u8 = 126;
        const READ_LENGTH: u8 = 10; // 2 bytes current + 4 bytes velocity + 4 bytes position

        let raw_data = self.controller
            .sync_read_raw_data(&MOTOR_IDS, CURRENT_ADDR, READ_LENGTH)
            .map_err(|e| anyhow::anyhow!("Failed to bulk read motor state (addr {}, len {}): {}",
                CURRENT_ADDR, READ_LENGTH, e))?;

        // Validate we got data for all motors
        if raw_data.len() != MOTOR_IDS.len() {
            return Err(anyhow::anyhow!("Incomplete motor data: expected {} motors, got {}",
                MOTOR_IDS.len(), raw_data.len()));
        }

        // Parse the 10-byte response for each motor
        for (i, motor_data) in raw_data.iter().enumerate() {
            if motor_data.len() != 10 {
                return Err(anyhow::anyhow!("Invalid data length for motor {} (ID {}): expected 10 bytes, got {}",
                    i, MOTOR_IDS[i], motor_data.len()));
            }

            // Extract current (first 2 bytes) as i16, convert to absolute mA
            let raw_current = i16::from_le_bytes([motor_data[0], motor_data[1]]);
            state.currents_ma[i] = (raw_current as f64).abs();

            // Extract velocity (next 4 bytes) as i32
            let raw_velocity = i32::from_le_bytes([
                motor_data[2], motor_data[3], motor_data[4], motor_data[5]
            ]);
            state.velocities[i] = convert_velocity(raw_velocity as f64);

            // Extract position (last 4 bytes) as i32 and convert to radians
            let raw_position = i32::from_le_bytes([
                motor_data[6], motor_data[7], motor_data[8], motor_data[9]
            ]);
            state.positions[i] = (2.0 * PI * (raw_position as f64) / 4096.0) - PI;
        }

        Ok(state)
    }

    /// Read current state using separate sync_reads (slower but more reliable)
    /// Uses three separate sync_read operations
    fn read_state_separate(&mut self) -> Result<MotorState> {
        let mut state = MotorState::default();

        // Sync read present position (in radians)
        let positions = self.controller
            .sync_read_present_position(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read positions: {}", e))?;
        state.positions.copy_from_slice(&positions);

        // Sync read present velocity (raw values need conversion)
        let raw_velocities = self.controller
            .sync_read_present_velocity(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read velocities: {}", e))?;
        for (i, &raw_vel) in raw_velocities.iter().enumerate() {
            state.velocities[i] = convert_velocity(raw_vel as f64);
        }

        // Sync read present current (in mA, absolute value)
        let raw_currents = self.controller
            .sync_read_present_current(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read currents: {}", e))?;
        for (i, &c) in raw_currents.iter().enumerate() {
            state.currents_ma[i] = (c as f64).abs();
        }

        Ok(state)
    }

    /// Write goal positions to all motors
    pub fn write_goal_positions(&mut self, positions: &[f64; NUM_MOTORS]) -> Result<()> {
        self.controller
            .sync_write_goal_position(&MOTOR_IDS, positions)
            .map_err(|e| anyhow::anyhow!("Failed to write goal positions: {}", e))?;
        Ok(())
    }

    /// Enable or disable torque for all motors
    pub fn set_torque_enable(&mut self, enable: bool) -> Result<()> {
        for &id in &MOTOR_IDS {
            self.controller
                .write_torque_enable(id, enable)
                .map_err(|e| anyhow::anyhow!("Failed to set torque for motor {}: {}", id, e))?;
        }
        Ok(())
    }

    /// Read current from all motors.
    /// Returns (left_leg_mA, right_leg_mA, total_all_motors_mA).
    /// left/right only sum leg motors (indices 0-4 and 9-13); total sums all 14.
    pub fn read_leg_currents(&mut self) -> Result<(f64, f64, f64)> {
        // Sync read present current (in mA)
        let currents = self.controller
            .sync_read_present_current(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read currents: {}", e))?;

        // Sum left leg currents (indices 0-4: IDs 20-24)
        let left_leg_current: f64 = currents[0..5].iter().map(|&c| (c as f64).abs()).sum();

        // Sum right leg currents (indices 9-13: IDs 10-14)
        let right_leg_current: f64 = currents[9..14].iter().map(|&c| (c as f64).abs()).sum();

        // Sum all 14 body motors
        let total_current: f64 = currents.iter().map(|&c| (c as f64).abs()).sum();

        Ok((left_leg_current, right_leg_current, total_current))
    }

    /// Read present input voltage for all motors (in Volts)
    pub fn read_voltages(&mut self) -> Result<[f32; NUM_MOTORS]> {
        let raw = self.controller
            .sync_read_present_input_voltage(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read voltages: {}", e))?;

        let mut voltages = [0.0f32; NUM_MOTORS];
        for (i, &v) in raw.iter().enumerate() {
            voltages[i] = v as f32 * 0.1;
        }
        Ok(voltages)
    }

    /// Initialize the mouth motor: enable torque and set PID gains
    pub fn init_mouth_motor(&mut self, kp: u16, ki: u16, kd: u16) -> Result<()> {
        self.controller
            .write_torque_enable(MOUTH_MOTOR_ID, true)
            .map_err(|e| anyhow::anyhow!("Failed to enable torque for mouth motor: {}", e))?;
        self.controller
            .write_position_p_gain(MOUTH_MOTOR_ID, kp)
            .map_err(|e| anyhow::anyhow!("Failed to set P gain for mouth motor: {}", e))?;
        self.controller
            .write_position_i_gain(MOUTH_MOTOR_ID, ki)
            .map_err(|e| anyhow::anyhow!("Failed to set I gain for mouth motor: {}", e))?;
        self.controller
            .write_position_d_gain(MOUTH_MOTOR_ID, kd)
            .map_err(|e| anyhow::anyhow!("Failed to set D gain for mouth motor: {}", e))?;
        Ok(())
    }

    /// Write goal position for the mouth motor (in radians)
    pub fn write_mouth_position(&mut self, pos_rad: f64) -> Result<()> {
        self.controller
            .sync_write_goal_position(&[MOUTH_MOTOR_ID], &[pos_rad])
            .map_err(|e| anyhow::anyhow!("Failed to write mouth position: {}", e))?;
        Ok(())
    }

    /// Smoothly interpolate all motors from their current positions to DEFAULT_POSITION.
    /// Reads current positions once, then linearly interpolates over `duration` at 50 Hz.
    pub fn interpolate_to_default(&mut self, duration: Duration) -> Result<()> {
        let steps = (duration.as_secs_f64() * 50.0).round() as u64;
        let dt = Duration::from_millis(20);

        // Read current positions as interpolation start
        let state = self.read_state()
            .context("Failed to read motor positions before interpolation")?;
        let start = state.positions;

        for step in 1..=steps {
            let t = step as f64 / steps as f64;
            let mut targets = [0.0f64; NUM_MOTORS];
            for i in 0..NUM_MOTORS {
                targets[i] = start[i] + t * (DEFAULT_POSITION[i] - start[i]);
            }
            self.write_goal_positions(&targets)
                .context("Failed to write interpolated positions")?;
            std::thread::sleep(dt);
        }
        Ok(())
    }

    /// Turn all motor LEDs on or off
    pub fn set_all_leds(&mut self, on: bool) -> Result<()> {
        let value: u8 = if on { 1 } else { 0 };
        let values = [value; NUM_MOTORS];
        self.controller
            .sync_write_led(&MOTOR_IDS, &values)
            .map_err(|e| anyhow::anyhow!("Failed to set LEDs: {}", e))?;
        Ok(())
    }

    /// Set PID gains for all motors
    pub fn set_pid_gains(&mut self, kp: u16, ki: u16, kd: u16) -> Result<()> {
        for &id in &MOTOR_IDS {
            self.controller
                .write_position_p_gain(id, kp)
                .map_err(|e| anyhow::anyhow!("Failed to set P gain for motor {}: {}", id, e))?;
            self.controller
                .write_position_i_gain(id, ki)
                .map_err(|e| anyhow::anyhow!("Failed to set I gain for motor {}: {}", id, e))?;
            self.controller
                .write_position_d_gain(id, kd)
                .map_err(|e| anyhow::anyhow!("Failed to set D gain for motor {}: {}", id, e))?;
        }
        Ok(())
    }
}

/// Convert raw velocity value to rad/s
/// Based on the conversion from ~/Rhoban/bam/bam/xl330/record.py
fn convert_velocity(raw_signed: f64) -> f64 {
    raw_signed * RADS_PER_SEC_PER_COUNT
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_conversion() {
        // Test zero velocity
        assert_eq!(convert_velocity(0.0), 0.0);

        // Test positive velocity
        let result = convert_velocity(100.0);
        assert!(result > 0.0);

        // Verify conversion matches the formula: 0.229 RPM * (2π/60) = rad/s
        // 100 counts = 100 * 0.229 = 22.9 RPM = 22.9 * 2π/60 ≈ 2.398 rad/s
        let expected = 100.0 * 0.229 * (2.0 * PI / 60.0);
        assert!((result - expected).abs() < 0.001);
    }

    #[test]
    fn test_position_conversion() {
        // Test zero position (raw value 2048 = center = 0 radians)
        let raw_center = 2048_i32;
        let position = (2.0 * PI * (raw_center as f64) / 4096.0) - PI;
        assert!((position - 0.0).abs() < 0.001);

        // Test min position (raw value 0 = -PI radians)
        let raw_min = 0_i32;
        let position_min = (2.0 * PI * (raw_min as f64) / 4096.0) - PI;
        assert!((position_min - (-PI)).abs() < 0.001);

        // Test max position (raw value 4095 ≈ PI radians)
        let raw_max = 4095_i32;
        let position_max = (2.0 * PI * (raw_max as f64) / 4096.0) - PI;
        assert!((position_max - PI).abs() < 0.002);
    }
}
