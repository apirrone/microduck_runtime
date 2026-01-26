use anyhow::{Context, Result};
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::f64::consts::PI;
use std::time::Duration;

/// Feature flag: use bulk read (faster but may be less reliable)
/// Set to false to use separate position/velocity reads (slower but more reliable)
const USE_BULK_READ: bool = true;

/// Number of motors on the microduck robot
pub const NUM_MOTORS: usize = 14;

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

/// Motor state containing position and velocity
#[derive(Debug, Clone)]
pub struct MotorState {
    pub positions: [f64; NUM_MOTORS],
    pub velocities: [f64; NUM_MOTORS],
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            positions: [0.0; NUM_MOTORS],
            velocities: [0.0; NUM_MOTORS],
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
    /// Uses a single optimized sync_read for both velocity and position
    fn read_state_bulk(&mut self) -> Result<MotorState> {
        let mut state = MotorState::default();

        // Optimized: Single sync read for both velocity (addr 128, 4 bytes) and position (addr 132, 4 bytes)
        // This reads 8 consecutive bytes starting at address 128
        const VELOCITY_ADDR: u8 = 128;
        const READ_LENGTH: u8 = 8;  // 4 bytes velocity + 4 bytes position

        let raw_data = self.controller
            .sync_read_raw_data(&MOTOR_IDS, VELOCITY_ADDR, READ_LENGTH)
            .map_err(|e| anyhow::anyhow!("Failed to bulk read motor state (addr {}, len {}): {}",
                VELOCITY_ADDR, READ_LENGTH, e))?;

        // Validate we got data for all motors
        if raw_data.len() != MOTOR_IDS.len() {
            return Err(anyhow::anyhow!("Incomplete motor data: expected {} motors, got {}",
                MOTOR_IDS.len(), raw_data.len()));
        }

        // Parse the 8-byte response for each motor
        for (i, motor_data) in raw_data.iter().enumerate() {
            if motor_data.len() != 8 {
                return Err(anyhow::anyhow!("Invalid data length for motor {} (ID {}): expected 8 bytes, got {}",
                    i, MOTOR_IDS[i], motor_data.len()));
            }

            // Extract velocity (first 4 bytes) as i32
            let raw_velocity = i32::from_le_bytes([
                motor_data[0], motor_data[1], motor_data[2], motor_data[3]
            ]);
            state.velocities[i] = convert_velocity(raw_velocity as f64);

            // Extract position (last 4 bytes) as i32 and convert to radians
            let raw_position = i32::from_le_bytes([
                motor_data[4], motor_data[5], motor_data[6], motor_data[7]
            ]);
            state.positions[i] = (2.0 * PI * (raw_position as f64) / 4096.0) - PI;
        }

        Ok(state)
    }

    /// Read current state using separate sync_reads (slower but more reliable)
    /// Uses two separate sync_read operations
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

    /// Read current from all motors and return total current for legs (in mA)
    /// Returns (left_leg_current_mA, right_leg_current_mA)
    pub fn read_leg_currents(&mut self) -> Result<(f64, f64)> {
        // Sync read present current (in mA)
        let currents = self.controller
            .sync_read_present_current(&MOTOR_IDS)
            .map_err(|e| anyhow::anyhow!("Failed to read currents: {}", e))?;

        // Sum left leg currents (indices 0-4: IDs 20-24)
        let left_leg_current: f64 = currents[0..5].iter().map(|&c| (c as f64).abs()).sum();

        // Sum right leg currents (indices 9-13: IDs 10-14)
        let right_leg_current: f64 = currents[9..14].iter().map(|&c| (c as f64).abs()).sum();

        Ok((left_leg_current, right_leg_current))
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
