use anyhow::{Context, Result};
use linux_embedded_hal::{Delay, I2cdev};
use bno055::{Bno055, AxisRemap, BNO055AxisConfig, BNO055AxisSign};

/// IMU data containing gyroscope and normalized projected gravity
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s (angular velocity in body frame)
    pub gyro: [f64; 3],
    /// Normalized projected gravity [x, y, z] (unit vector in body frame)
    /// Computed as: normalize(-accelerometer)
    /// When at rest upright: points down [0, 0, -1]
    pub accel: [f64; 3],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, -1.0], // Default normalized projected gravity (upright robot, unit vector pointing down)
        }
    }
}

/// IMU controller for BNO055 sensor using bno055 crate
pub struct ImuController {
    imu: Bno055<I2cdev>,
    delay: Delay,
}

impl ImuController {
    /// Create a new IMU controller with default settings
    /// Uses /dev/i2c-1 and alternative address (0x29)
    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", 0x29)
    }

    /// Create a new IMU controller
    /// i2c_device: path to I2C device (e.g., "/dev/i2c-1")
    /// address: BNO055 I2C address (0x28 or 0x29)
    pub fn new(i2c_device: &str, address: u8) -> Result<Self> {
        let i2c = I2cdev::new(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;

        let mut imu = if address == 0x29 {
            Bno055::new(i2c).with_alternative_address()
        } else {
            Bno055::new(i2c)
        };

        let mut delay = Delay;

        // Initialize BNO055
        imu.init(&mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to initialize BNO055: {:?}", e))?;

        // Configure hardware axis remapping
        // This matches test_imu3: Robot = [-Sensor_Y, -Sensor_X, Sensor_Z]
        let remap = AxisRemap::builder()
            .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)  // Swap X and Y axes
            .build()
            .map_err(|_| anyhow::anyhow!("Failed to build axis remap"))?;

        imu.set_axis_remap(remap)
            .map_err(|e| anyhow::anyhow!("Failed to set axis remap: {:?}", e))?;

        // Flip both X and Y axis signs
        imu.set_axis_sign(BNO055AxisSign::X_NEGATIVE | BNO055AxisSign::Y_NEGATIVE)
            .map_err(|e| anyhow::anyhow!("Failed to set axis sign: {:?}", e))?;

        // Set to NDOF mode
        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to set NDOF mode: {:?}", e))?;

        Ok(Self { imu, delay })
    }

    /// Get calibration status
    /// Returns (system, gyro, accelerometer, magnetometer) calibration levels (0-3)
    pub fn get_calibration_status(&mut self) -> Result<(u8, u8, u8, u8)> {
        let calib = self.imu.get_calibration_status()
            .map_err(|e| anyhow::anyhow!("Failed to get calibration status: {:?}", e))?;
        Ok((calib.sys, calib.gyr, calib.acc, calib.mag))
    }

    /// Read current IMU data
    /// Returns gyroscope (rad/s) and normalized projected gravity (unit vector)
    pub fn read(&mut self) -> Result<ImuData> {
        // Read accelerometer (hardware-remapped, in m/s²)
        let accel = self.imu.accel_data()
            .map_err(|e| anyhow::anyhow!("Failed to read accelerometer: {:?}", e))?;

        // Read gyroscope (hardware-remapped, in 1/16 deg/s)
        let gyro = self.imu.gyro_data()
            .map_err(|e| anyhow::anyhow!("Failed to read gyroscope: {:?}", e))?;

        // Convert accelerometer to f64 (already in m/s²)
        let accel_ms2 = [
            accel.x as f64,
            accel.y as f64,
            accel.z as f64,
        ];

        // Convert gyroscope to rad/s (from 1/16 deg/s)
        let gyro_scale = std::f64::consts::PI / (16.0 * 180.0);
        let gyro_rad_s = [
            gyro.x as f64 * gyro_scale,
            gyro.y as f64 * gyro_scale,
            gyro.z as f64 * gyro_scale,
        ];

        // Compute normalized projected gravity
        // Accelerometer measures normal force (pointing up when at rest)
        // Projected gravity is opposite direction (pointing down)
        let proj_grav_raw = [
            -accel_ms2[0],
            -accel_ms2[1],
            -accel_ms2[2],
        ];

        // Normalize to unit vector (MuJoCo expects normalized projected gravity)
        let mag = (proj_grav_raw[0].powi(2) + proj_grav_raw[1].powi(2) + proj_grav_raw[2].powi(2)).sqrt();
        let proj_grav = if mag > 0.1 {
            [
                proj_grav_raw[0] / mag,
                proj_grav_raw[1] / mag,
                proj_grav_raw[2] / mag,
            ]
        } else {
            [0.0, 0.0, -1.0] // Default to downward unit vector if magnitude is too small
        };

        Ok(ImuData {
            gyro: gyro_rad_s,
            accel: proj_grav,
        })
    }

    /// Read raw accelerometer data (for debugging)
    /// Returns accelerometer in m/s²
    pub fn read_raw_accelerometer(&mut self) -> Result<[f64; 3]> {
        let accel = self.imu.accel_data()
            .map_err(|e| anyhow::anyhow!("Failed to read accelerometer: {:?}", e))?;

        Ok([
            accel.x as f64,
            accel.y as f64,
            accel.z as f64,
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_data_default() {
        let data = ImuData::default();
        assert_eq!(data.gyro, [0.0, 0.0, 0.0]);
        assert_eq!(data.accel, [0.0, 0.0, -1.0]);
    }

    #[test]
    fn test_projected_gravity_upright() {
        // Test that default projected gravity points down when upright
        let data = ImuData::default();
        let mag = (data.accel[0].powi(2) + data.accel[1].powi(2) + data.accel[2].powi(2)).sqrt();
        assert!((mag - 1.0).abs() < 0.01, "Projected gravity should be unit vector");
        assert!(data.accel[2] < 0.0, "Projected gravity Z should be negative (pointing down)");
    }
}
