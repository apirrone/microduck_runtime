use anyhow::{Context, Result};
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::f64::consts::PI;
use std::time::Duration;

/// Feature flag: use bulk read (faster but may be less reliable)
/// Set to false to use separate position/velocity reads (slower but more reliable)
const USE_BULK_READ: bool = true;

/// Number of motors on the microduck robot (15: left leg + neck/head/mouth + right leg)
pub const NUM_MOTORS: usize = 15;

/// Index of the mouth motor in MOTOR_IDS / action vectors
pub const MOUTH_MOTOR_IDX: usize = 9;

/// Mouth motor Dynamixel ID
pub const MOUTH_MOTOR_ID: u8 = 34;

/// Mouth motor position range (radians): -10° to 70°
pub const MOUTH_MIN_ANGLE: f64 = -10.0 * std::f64::consts::PI / 180.0;
pub const MOUTH_MAX_ANGLE: f64 =  70.0 * std::f64::consts::PI / 180.0;

/// Motor IDs in order: left leg (5), neck/head/mouth (5), right leg (5)
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
    34, // [9]  mouth_motor
    10, // [10] right_hip_yaw
    11, // [11] right_hip_roll
    12, // [12] right_hip_pitch
    13, // [13] right_knee
    14, // [14] right_ankle
];

/// Default/initial position for all motors (bent legs position)
/// Order matches MOTOR_IDS: left leg (5), neck/head/mouth (5), right leg (5)
/// These values MUST match HOME_FRAME in microduck_constants.py for consistency with RL training
pub const DEFAULT_POSITION: [f64; NUM_MOTORS] = [
    0.0,  // left_hip_yaw
    0.0,  // left_hip_roll
    0.6,  // left_hip_pitch
    -1.2, // left_knee
    0.6,  // left_ankle

    -0.5, // neck_pitch      (override via --neck-pitch-default)
    0.5,  // head_pitch      (override via --head-pitch-default)
    0.0,  // head_yaw
    0.0,  // head_roll
    0.0,  // mouth_motor

    0.0,  // right_hip_yaw
    0.0,  // right_hip_roll
    -0.6, // right_hip_pitch
    1.2,  // right_knee
    -0.6, // right_ankle
];

/// Default position for the fold robot (STAND keyframe from fold/scene_fold.xml).
/// Motor order: left leg (5), neck/head/mouth (5), right leg (5).
/// Matches HOME_FRAME in fold_constants.py.
pub const FOLD_DEFAULT_POSITION: [f64; NUM_MOTORS] = [
    0.0,     // left_hip_yaw
    -0.1745, // left_hip_roll
    -0.5236, // left_hip_pitch
    0.0,     // left_knee
    -0.5236, // left_ankle

    -0.5236, // neck_pitch
    0.5236,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // mouth_motor (unused in fold policy)

    0.0,     // right_hip_yaw
    0.1745,  // right_hip_roll
    0.5236,  // right_hip_pitch
    0.0,     // right_knee
    0.5236,  // right_ankle
];

/// Conversion factor for velocity: 0.229 RPM per count * (2π / 60) for rad/s
const RADS_PER_SEC_PER_COUNT: f64 = 0.229 * (2.0 * PI / 60.0);

/// Dynamixel Operating Mode register values
pub const OPERATING_MODE_VELOCITY: u8 = 1;
pub const OPERATING_MODE_POSITION: u8 = 3;

/// Indices within MOTOR_IDS for the two wheel motors (left_ankle=4, right_ankle=14).
pub const WHEEL_MOTOR_INDICES: [usize; 2] = [4, 14];

/// Max wheel angular velocity [rad/s] — must match WHEEL_MAX_VEL in the training env.
pub const WHEEL_MAX_VEL: f64 = 10.0;

/// Default pose for motorized-wheel robot: 12 position-controlled joints.
/// Excludes wheel indices (4, 14) and mouth index (9).
/// Order: [left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee,
///          neck_pitch, head_pitch, head_yaw, head_roll,
///          right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee]
pub const DEFAULT_POSITION_MOTORIZED_WHEEL: [f64; 12] = [
    0.0,     // left_hip_yaw
    0.0,     // left_hip_roll
    0.5,     // left_hip_pitch
    -1.0,    // left_knee
    -0.3491, // neck_pitch
    0.3491,  // head_pitch
    0.0,     // head_yaw
    0.0,     // head_roll
    0.0,     // right_hip_yaw
    0.0,     // right_hip_roll
    -0.5,    // right_hip_pitch
    1.0,     // right_knee
];

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
    /// Whether motor 34 (mouth) responded at startup; false = skip it everywhere
    pub mouth_present: bool,
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

        let mut mc = Self { controller, mouth_present: true };
        // Probe motor 34 — if it doesn't respond it's absent; swallow the error
        match mc.controller.sync_read_present_position(&[MOUTH_MOTOR_ID]) {
            Ok(_) => println!("✓ Mouth motor (ID 34) present"),
            Err(_) => {
                mc.mouth_present = false;
                println!("⚠  Mouth motor (ID 34) not found — skipping in all motor operations");
            }
        }
        Ok(mc)
    }

    /// Returns (full_index, motor_id) pairs for all active motors.
    /// When mouth_present=false, motor 34 (index MOUTH_MOTOR_IDX) is excluded.
    fn active_motors(&self) -> Vec<(usize, u8)> {
        MOTOR_IDS.iter().copied().enumerate()
            .filter(|&(i, id)| self.mouth_present || id != MOUTH_MOTOR_ID)
            .collect()
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

        let active = self.active_motors();
        let ids: Vec<u8> = active.iter().map(|&(_, id)| id).collect();

        let raw_data = self.controller
            .sync_read_raw_data(&ids, CURRENT_ADDR, READ_LENGTH)
            .map_err(|e| anyhow::anyhow!("Failed to bulk read motor state (addr {}, len {}): {}",
                CURRENT_ADDR, READ_LENGTH, e))?;

        if raw_data.len() != active.len() {
            return Err(anyhow::anyhow!("Incomplete motor data: expected {} motors, got {}",
                active.len(), raw_data.len()));
        }

        // Parse each motor's 10-byte block; j = active index, i = full-array index
        for (j, motor_data) in raw_data.iter().enumerate() {
            let (i, motor_id) = active[j];
            if motor_data.len() != 10 {
                return Err(anyhow::anyhow!("Invalid data length for motor {} (ID {}): expected 10 bytes, got {}",
                    i, motor_id, motor_data.len()));
            }

            let raw_current = i16::from_le_bytes([motor_data[0], motor_data[1]]);
            state.currents_ma[i] = (raw_current as f64).abs();

            let raw_velocity = i32::from_le_bytes([
                motor_data[2], motor_data[3], motor_data[4], motor_data[5]
            ]);
            state.velocities[i] = convert_velocity(raw_velocity as f64);

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
        let active = self.active_motors();
        let ids: Vec<u8> = active.iter().map(|&(_, id)| id).collect();

        let positions = self.controller
            .sync_read_present_position(&ids)
            .map_err(|e| anyhow::anyhow!("Failed to read positions: {}", e))?;
        for (j, &pos) in positions.iter().enumerate() {
            state.positions[active[j].0] = pos;
        }

        let raw_velocities = self.controller
            .sync_read_present_velocity(&ids)
            .map_err(|e| anyhow::anyhow!("Failed to read velocities: {}", e))?;
        for (j, &raw_vel) in raw_velocities.iter().enumerate() {
            state.velocities[active[j].0] = convert_velocity(raw_vel as f64);
        }

        let raw_currents = self.controller
            .sync_read_present_current(&ids)
            .map_err(|e| anyhow::anyhow!("Failed to read currents: {}", e))?;
        for (j, &c) in raw_currents.iter().enumerate() {
            state.currents_ma[active[j].0] = (c as f64).abs();
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
        for &(_, id) in &self.active_motors() {
            self.controller
                .write_torque_enable(id, enable)
                .map_err(|e| anyhow::anyhow!("Failed to set torque for motor {}: {}", id, e))?;
        }
        Ok(())
    }

    /// Read current from all motors.
    /// Returns (left_leg_mA, right_leg_mA, total_all_motors_mA).
    /// left/right only sum leg motors (indices 0-4 and 10-14); total sums all active motors.
    pub fn read_leg_currents(&mut self) -> Result<(f64, f64, f64)> {
        let active = self.active_motors();
        let ids: Vec<u8> = active.iter().map(|&(_, id)| id).collect();
        let raw = self.controller
            .sync_read_present_current(&ids)
            .map_err(|e| anyhow::anyhow!("Failed to read currents: {}", e))?;

        let mut currents_ma = [0.0f64; NUM_MOTORS];
        for (j, &c) in raw.iter().enumerate() {
            currents_ma[active[j].0] = (c as f64).abs();
        }

        let left_leg_current:  f64 = currents_ma[0..5].iter().sum();
        let right_leg_current: f64 = currents_ma[10..15].iter().sum();
        let total_current:     f64 = currents_ma.iter().sum();

        Ok((left_leg_current, right_leg_current, total_current))
    }

    /// Read present input voltage for all motors (in Volts)
    pub fn read_voltages(&mut self) -> Result<[f32; NUM_MOTORS]> {
        let active = self.active_motors();
        let ids: Vec<u8> = active.iter().map(|&(_, id)| id).collect();
        let raw = self.controller
            .sync_read_present_input_voltage(&ids)
            .map_err(|e| anyhow::anyhow!("Failed to read voltages: {}", e))?;

        let mut voltages = [0.0f32; NUM_MOTORS];
        for (j, &v) in raw.iter().enumerate() {
            voltages[active[j].0] = v as f32 * 0.1;
        }
        Ok(voltages)
    }

    /// Smoothly interpolate all motors from their current positions to DEFAULT_POSITION.
    /// Reads current positions once, then linearly interpolates over `duration` at 50 Hz.
    pub fn interpolate_to_default(&mut self, duration: Duration) -> Result<()> {
        self.interpolate_to_position(&DEFAULT_POSITION, duration)
    }

    /// Smoothly interpolate all motors from their current positions to a given target.
    /// Reads current positions once, then linearly interpolates over `duration` at 50 Hz.
    pub fn interpolate_to_position(&mut self, target: &[f64; NUM_MOTORS], duration: Duration) -> Result<()> {
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
                targets[i] = start[i] + t * (target[i] - start[i]);
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

    /// Check and correct EEPROM configuration registers for all motors.
    /// Verifies: return_delay_time=0, baud_rate=3 (1 Mbps), homing_offset=0, pwm_slope=255, shutdown=52.
    /// Writes any register that doesn't match. Torque must be disabled before calling.
    pub fn check_and_fix_config(&mut self) -> Result<()> {
        let mut total_fixes = 0usize;

        let active_ids: Vec<u8> = self.active_motors().into_iter().map(|(_, id)| id).collect();
        for id in active_ids {
            total_fixes += self.check_and_fix_motor_config(id)
                .map_err(|e| anyhow::anyhow!("Motor {}: {}", id, e))?;
        }

        if total_fixes == 0 {
            println!("✓ All motor config registers correct");
        } else {
            println!("✓ Motor config: corrected {} register(s)", total_fixes);
        }

        Ok(())
    }

    fn check_and_fix_motor_config(&mut self, id: u8) -> Result<usize> {
        let mut fixes = 0usize;

        // return_delay_time: 0
        let val = self.controller.read_return_delay_time(id)
            .map_err(|e| anyhow::anyhow!("read return_delay_time: {}", e))?[0];
        if val != 0 {
            println!("  Motor {}: fixing return_delay_time {} -> 0", id, val);
            self.controller.write_return_delay_time(id, 0u8)
                .map_err(|e| anyhow::anyhow!("write return_delay_time: {}", e))?;
            fixes += 1;
        }

        // baud_rate register: 3 = 1,000,000 bps
        let val = self.controller.read_baud_rate(id)
            .map_err(|e| anyhow::anyhow!("read baud_rate: {}", e))?[0];
        if val != 3 {
            println!("  Motor {}: fixing baud_rate {} -> 3 (1000000)", id, val);
            self.controller.write_baud_rate(id, 3u8)
                .map_err(|e| anyhow::anyhow!("write baud_rate: {}", e))?;
            fixes += 1;
        }

        // homing_offset: 0
        let val = self.controller.read_homing_offset(id)
            .map_err(|e| anyhow::anyhow!("read homing_offset: {}", e))?[0];
        if val != 0 {
            println!("  Motor {}: fixing homing_offset {} -> 0", id, val);
            self.controller.write_homing_offset(id, 0i32)
                .map_err(|e| anyhow::anyhow!("write homing_offset: {}", e))?;
            fixes += 1;
        }

        // pwm_slope: 255
        let val = self.controller.read_pwm_slope(id)
            .map_err(|e| anyhow::anyhow!("read pwm_slope: {}", e))?[0];
        if val != 255 {
            println!("  Motor {}: fixing pwm_slope {} -> 255", id, val);
            self.controller.write_pwm_slope(id, 255u8)
                .map_err(|e| anyhow::anyhow!("write pwm_slope: {}", e))?;
            fixes += 1;
        }

        // shutdown: 52
        let val = self.controller.read_shutdown(id)
            .map_err(|e| anyhow::anyhow!("read shutdown: {}", e))?[0];
        if val != 52 {
            println!("  Motor {}: fixing shutdown {} -> 52", id, val);
            self.controller.write_shutdown(id, 52u8)
                .map_err(|e| anyhow::anyhow!("write shutdown: {}", e))?;
            fixes += 1;
        }

        Ok(fixes)
    }

    /// Set PID gains for all motors
    pub fn set_pid_gains(&mut self, kp: u16, ki: u16, kd: u16) -> Result<()> {
        for &(_, id) in &self.active_motors() {
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

    /// Write the operating_mode EEPROM register for a single motor.
    /// Torque must be disabled before calling this.
    pub fn set_operating_mode(&mut self, motor_id: u8, mode: u8) -> Result<()> {
        self.controller
            .write_operating_mode(motor_id, mode)
            .map_err(|e| anyhow::anyhow!("Failed to write operating_mode for motor {}: {}", motor_id, e))?;
        Ok(())
    }

    /// Switch the two wheel motors (IDs at WHEEL_MOTOR_INDICES) to velocity control (Op Mode 1).
    /// Disables torque → writes mode → re-enables torque.
    pub fn set_wheel_motors_velocity_mode(&mut self) -> Result<()> {
        let wheel_ids: Vec<u8> = WHEEL_MOTOR_INDICES.iter().map(|&i| MOTOR_IDS[i]).collect();
        for &id in &wheel_ids {
            self.controller.write_torque_enable(id, false)
                .map_err(|e| anyhow::anyhow!("Failed to disable torque for wheel motor {}: {}", id, e))?;
            self.set_operating_mode(id, OPERATING_MODE_VELOCITY)?;
            self.controller.write_torque_enable(id, true)
                .map_err(|e| anyhow::anyhow!("Failed to re-enable torque for wheel motor {}: {}", id, e))?;
        }
        Ok(())
    }

    /// Restore the two wheel motors to position control (Op Mode 3).
    /// Disables torque → writes mode → re-enables torque.
    pub fn set_wheel_motors_position_mode(&mut self) -> Result<()> {
        let wheel_ids: Vec<u8> = WHEEL_MOTOR_INDICES.iter().map(|&i| MOTOR_IDS[i]).collect();
        for &id in &wheel_ids {
            self.controller.write_torque_enable(id, false)
                .map_err(|e| anyhow::anyhow!("Failed to disable torque for wheel motor {}: {}", id, e))?;
            self.set_operating_mode(id, OPERATING_MODE_POSITION)?;
            self.controller.write_torque_enable(id, true)
                .map_err(|e| anyhow::anyhow!("Failed to re-enable torque for wheel motor {}: {}", id, e))?;
        }
        Ok(())
    }

    /// Read present velocities of the two wheel motors only [rad/s].
    /// Uses a dedicated sync_read_present_velocity on just the two wheel IDs,
    /// avoiding the full bulk read which can fail in velocity control mode.
    /// Returns (left_vel_rad_s, right_vel_rad_s).
    pub fn read_wheel_velocities(&mut self) -> Result<(f64, f64)> {
        let wheel_ids = [MOTOR_IDS[WHEEL_MOTOR_INDICES[0]], MOTOR_IDS[WHEEL_MOTOR_INDICES[1]]];
        let raw = self.controller
            .sync_read_present_velocity(&wheel_ids)
            .map_err(|e| anyhow::anyhow!("Failed to read wheel velocities: {}", e))?;
        Ok((
            convert_velocity(raw[0] as f64),
            convert_velocity(raw[1] as f64),
        ))
    }

    /// Write goal velocities to the two wheel motors.
    /// `left_vel_rad_s` and `right_vel_rad_s` are in rad/s.
    pub fn write_wheel_velocities(&mut self, left_vel_rad_s: f64, right_vel_rad_s: f64) -> Result<()> {
        let left_id  = MOTOR_IDS[WHEEL_MOTOR_INDICES[0]];
        let right_id = MOTOR_IDS[WHEEL_MOTOR_INDICES[1]];
        let left_raw  = (left_vel_rad_s  / RADS_PER_SEC_PER_COUNT).round() as i32;
        let right_raw = (right_vel_rad_s / RADS_PER_SEC_PER_COUNT).round() as i32;
        self.controller
            .sync_write_goal_velocity(&[left_id, right_id], &[left_raw, right_raw])
            .map_err(|e| anyhow::anyhow!("Failed to write wheel velocities: {}", e))?;
        Ok(())
    }

    /// Write goal positions to all motors except the two wheel motors.
    /// `positions` is the full NUM_MOTORS array; wheel indices are skipped.
    pub fn write_goal_positions_no_wheels(&mut self, positions: &[f64; NUM_MOTORS]) -> Result<()> {
        let mut ids: Vec<u8>  = Vec::with_capacity(NUM_MOTORS - WHEEL_MOTOR_INDICES.len());
        let mut pos: Vec<f64> = Vec::with_capacity(NUM_MOTORS - WHEEL_MOTOR_INDICES.len());
        for i in 0..NUM_MOTORS {
            if !WHEEL_MOTOR_INDICES.contains(&i) {
                ids.push(MOTOR_IDS[i]);
                pos.push(positions[i]);
            }
        }
        self.controller
            .sync_write_goal_position(&ids, &pos)
            .map_err(|e| anyhow::anyhow!("Failed to write goal positions (no wheels): {}", e))?;
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
