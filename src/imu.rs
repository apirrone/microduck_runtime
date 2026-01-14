use anyhow::Result;

/// IMU data containing gyroscope and accelerometer readings
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s
    pub gyro: [f64; 3],
    /// Accelerometer data [x, y, z] in m/s²
    pub accel: [f64; 3],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81], // Default to gravity in z-axis
        }
    }
}

/// IMU controller for BNO055 sensor
/// Currently a dummy implementation, to be replaced with actual BNO055 driver
pub struct ImuController {
    // Placeholder for future BNO055 device handle
    _dummy: (),
}

impl ImuController {
    /// Create a new IMU controller
    /// TODO: Add actual BNO055 initialization when sensor is connected
    pub fn new() -> Result<Self> {
        Ok(Self { _dummy: () })
    }

    /// Read current IMU data
    /// TODO: Replace with actual BNO055 readings
    pub fn read(&mut self) -> Result<ImuData> {
        // Return default values for now
        // When implementing:
        // - Read from BNO055 via I2C
        // - Convert to appropriate units (rad/s for gyro, m/s² for accel)
        // - Apply any necessary calibration
        Ok(ImuData::default())
    }
}

impl Default for ImuController {
    fn default() -> Self {
        Self::new().unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_dummy() {
        let mut imu = ImuController::new().unwrap();
        let data = imu.read().unwrap();

        // Check default gravity
        assert_eq!(data.accel[2], 9.81);
    }
}
