use crate::imu::ImuData;
use crate::motor::{MotorState, NUM_MOTORS, DEFAULT_POSITION};

/// Maximum observation size (with phase)
pub const MAX_OBSERVATION_SIZE: usize = 53;

/// Observation vector structure
/// Layout: [gyro(3), projected_gravity(3), joint_pos(14), joint_vel(14), last_action(14), command(3), phase(2)?]
/// - gyro: angular velocity in body frame (rad/s)
/// - projected_gravity: normalized gravity vector in body frame (unit vector)
/// - joint_pos: relative to DEFAULT_POSITION (rad)
/// - joint_vel: joint velocities (rad/s)
/// - last_action: previous action outputs (rad offsets)
/// - command: [lin_vel_x, lin_vel_y, ang_vel_z] velocity commands
/// - phase: [cos(phase*2π), sin(phase*2π)] - optional imitation phase (2D)
#[derive(Debug, Clone)]
pub struct Observation {
    data: [f32; MAX_OBSERVATION_SIZE],
    size: usize,
}

impl Observation {
    /// Create a new observation from sensor data
    pub fn new(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        phase: Option<f64>,
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3) - base_ang_vel in RL env
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Projected gravity (3) - gravity vector in body frame
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Joint positions relative to default position (14)
        for i in 0..NUM_MOTORS {
            data[idx] = (motor_state.positions[i] - DEFAULT_POSITION[i]) as f32;
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

        // Imitation phase (2) - [cos, sin] (optional)
        if let Some(phase_val) = phase {
            let angle = phase_val * 2.0 * std::f64::consts::PI;
            data[idx] = angle.cos() as f32;
            idx += 1;
            data[idx] = angle.sin() as f32;
            idx += 1;
        }

        let size = idx;
        assert!(size == 51 || size == 53, "Observation size must be 51 or 53, got {}", size);

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
            size: 51,  // Default without phase
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::MotorState;

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

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, None);
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

        // Check command (now at the end: 3 + 3 + 14 + 14 + 14 = 48)
        assert_eq!(slice[48], 0.1);
        assert_eq!(slice[49], 0.2);
        assert_eq!(slice[50], 0.3);
    }

    #[test]
    fn test_observation_with_phase() {
        let imu = ImuData {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, -1.0],
        };
        let command = [0.0; 3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        // Test with phase = 0.0
        let obs = Observation::new(&imu, &command, &motor_state, &last_action, Some(0.0));
        assert_eq!(obs.size(), 53);
        let slice = obs.as_slice();
        // phase = 0 -> cos(0) = 1, sin(0) = 0
        assert!((slice[51] - 1.0).abs() < 0.001);
        assert!((slice[52] - 0.0).abs() < 0.001);

        // Test with phase = 0.25
        let obs = Observation::new(&imu, &command, &motor_state, &last_action, Some(0.25));
        assert_eq!(obs.size(), 53);
        let slice = obs.as_slice();
        // phase = 0.25 -> cos(π/2) = 0, sin(π/2) = 1
        assert!((slice[51] - 0.0).abs() < 0.001);
        assert!((slice[52] - 1.0).abs() < 0.001);
    }
}
