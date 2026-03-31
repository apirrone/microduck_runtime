use crate::imu::ImuData;
use crate::motor::{MotorState, NUM_MOTORS};

/// Maximum observation size (with phase)
pub const MAX_OBSERVATION_SIZE: usize = 56;

/// Observation vector structure
///
/// Layout: [gyro(3), projected_gravity(3), joint_pos(14), joint_vel(14), last_action(14), command(3)]
/// Total: 51D
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
    pub fn new(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3) - base_ang_vel in RL env
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Projected gravity OR raw accelerometer (3) - depends on --projected-gravity flag
        // With --projected-gravity: pure gravity direction (quaternion-based, no dynamics)
        // Without flag: raw accelerometer (includes gravity + linear acceleration + dynamics)
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Joint positions relative to default position (14)
        for i in 0..NUM_MOTORS {
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }

        // Joint velocities (14)
        for i in 0..NUM_MOTORS {
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }

        // Last action (14)
        for i in 0..NUM_MOTORS {
            data[idx] = last_action[i];
            idx += 1;
        }

        // Velocity commands (3) - [lin_vel_x, lin_vel_y, ang_vel_z]
        for i in 0..3 {
            data[idx] = command[i] as f32;
            idx += 1;
        }

        let size = idx;
        assert!(size == 51, "Observation size must be 51, got {}", size);

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
            size: 51,  // Default velocity task size
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::{MotorState, DEFAULT_POSITION};

    #[test]
    fn test_observation_size() {
        let obs = Observation::default();
        assert_eq!(obs.as_slice().len(), 51);
        assert_eq!(obs.size(), 51);
    }

    #[test]
    fn test_observation_creation() {
        let imu = ImuData {
            gyro: [1.0, 2.0, 3.0],
            accel: [4.0, 5.0, 6.0],
        };
        let command = [0.1, 0.2, 0.3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, &DEFAULT_POSITION);
        let slice = obs.as_slice();

        assert_eq!(obs.size(), 51);

        // Check gyro
        assert_eq!(slice[0], 1.0);
        assert_eq!(slice[1], 2.0);
        assert_eq!(slice[2], 3.0);

        // Check accel
        assert_eq!(slice[3], 4.0);
        assert_eq!(slice[4], 5.0);
        assert_eq!(slice[5], 6.0);

        // Check command (at the end: 3 + 3 + 14 + 14 + 14 = 48)
        assert_eq!(slice[48], 0.1);
        assert_eq!(slice[49], 0.2);
        assert_eq!(slice[50], 0.3);
    }

}
