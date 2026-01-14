use crate::imu::ImuData;
use crate::motor::{MotorState, NUM_MOTORS, DEFAULT_POSITION};

/// Size of the observation vector
pub const OBSERVATION_SIZE: usize = 51;

/// Observation vector structure
/// Layout: [gyro(3), accel(3), command(3), joint_pos(14), joint_vel(14), last_action(14)]
#[derive(Debug, Clone)]
pub struct Observation {
    data: [f32; OBSERVATION_SIZE],
}

impl Observation {
    /// Create a new observation from sensor data
    pub fn new(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
    ) -> Self {
        let mut data = [0.0f32; OBSERVATION_SIZE];
        let mut idx = 0;

        // Gyroscope (3)
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Accelerometer (3)
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Command (3)
        for i in 0..3 {
            data[idx] = command[i] as f32;
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

        assert_eq!(idx, OBSERVATION_SIZE);

        Self { data }
    }

    /// Get the observation as a slice
    pub fn as_slice(&self) -> &[f32] {
        &self.data
    }

    /// Get mutable access to the observation data
    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.data
    }
}

impl Default for Observation {
    fn default() -> Self {
        Self {
            data: [0.0; OBSERVATION_SIZE],
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
        assert_eq!(obs.as_slice().len(), OBSERVATION_SIZE);
        assert_eq!(OBSERVATION_SIZE, 3 + 3 + 3 + 14 + 14 + 14);
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

        let obs = Observation::new(&imu, &command, &motor_state, &last_action);
        let slice = obs.as_slice();

        // Check gyro
        assert_eq!(slice[0], 1.0);
        assert_eq!(slice[1], 2.0);
        assert_eq!(slice[2], 3.0);

        // Check accel
        assert_eq!(slice[3], 4.0);
        assert_eq!(slice[4], 5.0);
        assert_eq!(slice[5], 6.0);

        // Check command
        assert_eq!(slice[6], 0.1);
        assert_eq!(slice[7], 0.2);
        assert_eq!(slice[8], 0.3);
    }
}
