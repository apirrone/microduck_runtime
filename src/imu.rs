use anyhow::{Context, Result};
use bno055::{BNO055OperationMode, Bno055};
use linux_embedded_hal::{Delay, I2cdev};
use std::thread;
use std::time::Duration;

/// IMU data containing gyroscope and projected gravity
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s (angular velocity in body frame)
    pub gyro: [f64; 3],
    /// Projected gravity [x, y, z] in m/s² (gravity vector rotated to body frame)
    /// This is computed from orientation quaternion: R^T * [0, 0, -9.81]
    /// NOT raw accelerometer (which includes linear acceleration)
    pub accel: [f64; 3],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, -9.81], // Default projected gravity (upright robot)
        }
    }
}

/// Rotate a vector by a quaternion
/// quat = [w, x, y, z], vec = [x, y, z]
fn quat_rotate_vec(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
    let [w, qx, qy, qz] = quat;
    let [vx, vy, vz] = vec;

    // Quaternion rotation: v' = q * v * q^-1
    // Optimized formula: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
    let cx = qy * vz - qz * vy;
    let cy = qz * vx - qx * vz;
    let cz = qx * vy - qy * vx;

    let cx2 = cy * qz - cz * qy + w * cx;
    let cy2 = cz * qx - cx * qz + w * cy;
    let cz2 = cx * qy - cy * qx + w * cz;

    [
        vx + 2.0 * cx2,
        vy + 2.0 * cy2,
        vz + 2.0 * cz2,
    ]
}

/// IMU controller for BNO055 sensor
pub struct ImuController {
    bno: Bno055<I2cdev>,
}

impl ImuController {
    /// Create a new IMU controller
    /// i2c_device: path to I2C device (e.g., "/dev/i2c-1")
    /// address: BNO055 I2C address (0x28 or 0x29)
    pub fn new(i2c_device: &str, address: u8) -> Result<Self> {
        let i2c = I2cdev::new(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;

        let mut bno = if address == 0x29 {
            Bno055::new(i2c).with_alternative_address()
        } else {
            Bno055::new(i2c)
        };

        let mut delay = Delay;

        bno.init(&mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to initialize BNO055: {:?}", e))?;

        // Set to NDOF mode (9-axis fusion with fast magnetometer calibration)
        bno.set_mode(BNO055OperationMode::NDOF, &mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to set BNO055 mode: {:?}", e))?;

        // Give the sensor time to switch modes
        thread::sleep(Duration::from_millis(100));

        Ok(Self { bno })
    }

    /// Create a new IMU controller with default settings
    /// Uses /dev/i2c-1 and address 0x28
    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", 0x28)
    }

    /// Read current IMU data
    pub fn read(&mut self) -> Result<ImuData> {
        // Read gyroscope (angular velocity) in rad/s
        let gyro_data = self.bno
            .gyro_data()
            .map_err(|e| anyhow::anyhow!("Failed to read gyro: {:?}", e))?;

        let gyro = [
            gyro_data.x as f64,
            gyro_data.y as f64,
            gyro_data.z as f64,
        ];

        // Read orientation quaternion [w, x, y, z]
        let quat_data = self.bno
            .quaternion()
            .map_err(|e| anyhow::anyhow!("Failed to read quaternion: {:?}", e))?;

        // BNO055 quaternion format: s (scalar/w) + v (vector x,y,z)
        let quat = [
            quat_data.s as f64,      // w
            quat_data.v.x as f64,    // x
            quat_data.v.y as f64,    // y
            quat_data.v.z as f64,    // z
        ];

        // Compute projected gravity: rotate world gravity [0, 0, -9.81] to body frame
        // Use conjugate quaternion for inverse rotation: [w, -x, -y, -z]
        let quat_conj = [quat[0], -quat[1], -quat[2], -quat[3]];
        let world_gravity = [0.0, 0.0, -9.81];
        let projected_gravity = quat_rotate_vec(quat_conj, world_gravity);

        Ok(ImuData {
            gyro,
            accel: projected_gravity,
        })
    }

    /// Get calibration status
    /// Returns (system, gyro, accel, mag) calibration levels (0-3)
    pub fn get_calibration_status(&mut self) -> Result<(u8, u8, u8, u8)> {
        let status = self.bno
            .get_calibration_status()
            .map_err(|e| anyhow::anyhow!("Failed to read calibration status: {:?}", e))?;

        Ok((status.sys, status.gyr, status.acc, status.mag))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_data_default() {
        let data = ImuData::default();

        // Check default projected gravity (upright robot)
        assert_eq!(data.accel[2], -9.81);
        assert_eq!(data.gyro[0], 0.0);
    }

    #[test]
    fn test_quat_rotate_identity() {
        // Identity quaternion [1, 0, 0, 0] should not rotate
        let quat = [1.0, 0.0, 0.0, 0.0];
        let vec = [1.0, 2.0, 3.0];
        let result = quat_rotate_vec(quat, vec);

        assert!((result[0] - vec[0]).abs() < 1e-10);
        assert!((result[1] - vec[1]).abs() < 1e-10);
        assert!((result[2] - vec[2]).abs() < 1e-10);
    }

    #[test]
    fn test_projected_gravity_upright() {
        // Upright robot: identity rotation
        let quat = [1.0, 0.0, 0.0, 0.0];
        let world_gravity = [0.0, 0.0, -9.81];
        let projected = quat_rotate_vec(quat, world_gravity);

        // Should match world gravity
        assert!((projected[2] - (-9.81)).abs() < 1e-10);
    }
}
