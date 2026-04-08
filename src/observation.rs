use crate::imu::ImuData;
use crate::motor::{MotorState, NUM_MOTORS, MOUTH_MOTOR_IDX};

/// Maximum observation size (with phase)
pub const MAX_OBSERVATION_SIZE: usize = 60;

/// Observation vector structure
///
/// Layout: [gyro(3), projected_gravity(3), joint_pos(15), joint_vel(15), last_action(15), command(3)]
/// Total: 54D  (15 joints: left leg(5) + neck/head/mouth(5) + right leg(5))
///
/// - gyro: angular velocity in body frame (rad/s)
/// - projected_gravity: normalized gravity vector in body frame (unit vector)
/// - joint_pos: relative to DEFAULT_POSITION (rad)
/// - joint_vel: joint velocities (rad/s)
/// - last_action: previous action outputs (rad offsets)
/// - command: [lin_vel_x, lin_vel_y, ang_vel_z] velocity commands
#[derive(Debug, Clone)]
pub struct Observation {
    data: [f32; MAX_OBSERVATION_SIZE],
    size: usize,
}

impl Observation {
    /// Create a new observation from sensor data.
    ///
    /// When `mouth_enabled` is true, all 15 joints are included → 54D observation.
    /// When false, the mouth joint (index MOUTH_MOTOR_IDX) is skipped → 51D observation,
    /// for backward compatibility with 14-joint policies.
    pub fn new(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
        mouth_enabled: bool,
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3)
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Projected gravity / raw accelerometer (3)
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Joint positions relative to default position
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }

        // Joint velocities
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }

        // Last action
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = last_action[i];
            idx += 1;
        }

        // Velocity commands (3)
        for i in 0..3 {
            data[idx] = command[i] as f32;
            idx += 1;
        }

        let size = idx;
        let expected = if mouth_enabled { 54 } else { 51 };
        assert!(size == expected, "Observation size must be {}, got {}", expected, size);

        Self { data, size }
    }

    /// Get the observation as a slice
    pub fn as_slice(&self) -> &[f32] {
        &self.data[..self.size]
    }

    /// Get the observation size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get mutable access to the observation data
    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.data[..self.size]
    }
}

impl Default for Observation {
    fn default() -> Self {
        Self {
            data: [0.0; MAX_OBSERVATION_SIZE],
            size: 51,  // Default: legacy 14-joint mode
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::{MotorState, DEFAULT_POSITION};

    #[test]
    fn test_observation_size_legacy() {
        let obs = Observation::default();
        assert_eq!(obs.as_slice().len(), 51);
        assert_eq!(obs.size(), 51);
    }

    #[test]
    fn test_observation_creation_legacy() {
        let imu = ImuData { gyro: [1.0, 2.0, 3.0], accel: [4.0, 5.0, 6.0], ..ImuData::default() };
        let command = [0.1, 0.2, 0.3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, &DEFAULT_POSITION, false);
        let slice = obs.as_slice();

        assert_eq!(obs.size(), 51);
        assert_eq!(slice[0], 1.0); // gyro x
        assert_eq!(slice[3], 4.0); // accel x
        // command at end: 3 + 3 + 14 + 14 + 14 = 48
        assert_eq!(slice[48], 0.1);
        assert_eq!(slice[49], 0.2);
        assert_eq!(slice[50], 0.3);
    }

    #[test]
    fn test_observation_creation_mouth() {
        let imu = ImuData { gyro: [1.0, 2.0, 3.0], accel: [4.0, 5.0, 6.0], ..ImuData::default() };
        let command = [0.1, 0.2, 0.3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, &DEFAULT_POSITION, true);
        let slice = obs.as_slice();

        assert_eq!(obs.size(), 54);
        // command at end: 3 + 3 + 15 + 15 + 15 = 51
        assert_eq!(slice[51], 0.1);
        assert_eq!(slice[52], 0.2);
        assert_eq!(slice[53], 0.3);
    }

}
