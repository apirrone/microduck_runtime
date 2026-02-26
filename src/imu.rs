use anyhow::{Context, Result};
use linux_embedded_hal::{Delay, I2cdev};
use bno055::{Bno055, BNO055AxisSign, BNO055Calibration};
use std::fs;
use std::path::PathBuf;

/// IMU data containing gyroscope and normalized projected gravity
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s (angular velocity in body frame)
    pub gyro: [f64; 3],
    /// Normalized "projected gravity" from raw accelerometer [x, y, z] (unit vector in body frame)
    /// Computed from raw accelerometer reading (includes gravity + linear acceleration).
    /// This is what a real IMU sensor measures: normalized(-accelerometer).
    /// Includes dynamic accelerations, impacts, and vibrations (unlike pure projected gravity).
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
    /// Gravity calibration offset to apply (subtracted from computed gravity)
    /// This compensates for IMU mounting angle or calibration bias
    /// Determined by measuring gravity when robot is standing upright
    gravity_offset: [f64; 3],
    /// Use projected gravity from quaternion (true) or raw accelerometer (false)
    use_projected_gravity: bool,
    /// Last 2 raw gyro readings for 3-sample median filter (spike rejection)
    gyro_history: [[f64; 3]; 2],
    /// Last 2 raw accel readings for 3-sample median filter (spike rejection)
    accel_history: [[f64; 3]; 2],
}

impl ImuController {
    /// Create a new IMU controller with default settings
    /// Uses /dev/i2c-1 and alternative address (0x29)
    /// use_projected_gravity: false = raw accelerometer, true = quaternion-based projected gravity
    pub fn new_default_with_mode(use_projected_gravity: bool) -> Result<Self> {
        Self::new("/dev/i2c-1", 0x29, use_projected_gravity)
    }

    /// Create a new IMU controller with default settings (raw accelerometer mode)
    /// Uses /dev/i2c-1 and alternative address (0x29)
    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", 0x29, false)
    }

    /// Create a new IMU controller
    /// i2c_device: path to I2C device (e.g., "/dev/i2c-1")
    /// address: BNO055 I2C address (0x28 or 0x29)
    /// use_projected_gravity: false = raw accelerometer, true = quaternion-based projected gravity
    pub fn new(i2c_device: &str, address: u8, use_projected_gravity: bool) -> Result<Self> {
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
        // IMU mounting: 90° clockwise yaw relative to previous orientation (viewed from above)
        // Previous: Sensor X+ → right,     Sensor Y+ → forward
        // New:      Sensor X+ → backward,  Sensor Y+ → right
        // Robot frame: X+ forward, Y+ left, Z+ up
        // Mapping: Robot = [-Sensor_X, -Sensor_Y, +Sensor_Z]
        imu.set_axis_sign(BNO055AxisSign::X_NEGATIVE | BNO055AxisSign::Y_NEGATIVE)
            .map_err(|e| anyhow::anyhow!("Failed to set axis sign: {:?}", e))?;

        // Set to IMU mode (6-axis fusion: accelerometer + gyroscope, no magnetometer)
        imu.set_mode(bno055::BNO055OperationMode::IMU, &mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to set IMU mode: {:?}", e))?;

        let mut controller = Self {
            imu,
            delay,
            gravity_offset: [0.0; 3],  // No offset by default
            use_projected_gravity,
            gyro_history: [[0.0; 3]; 2],
            accel_history: [[0.0; 3]; 2],
        };

        // Automatically load calibration if it exists
        match controller.load_calibration() {
            Ok(true) => {
                // Calibration loaded successfully
            }
            Ok(false) => {
                // No calibration file found - sensor will auto-calibrate
            }
            Err(e) => {
                eprintln!("Warning: Failed to load IMU calibration: {}", e);
                eprintln!("Sensor will use auto-calibration.");
            }
        }

        Ok(controller)
    }

    /// Get calibration status
    /// Returns (system, gyro, accelerometer, magnetometer) calibration levels (0-3)
    pub fn get_calibration_status(&mut self) -> Result<(u8, u8, u8, u8)> {
        let calib = self.imu.get_calibration_status()
            .map_err(|e| anyhow::anyhow!("Failed to get calibration status: {:?}", e))?;
        Ok((calib.sys, calib.gyr, calib.acc, calib.mag))
    }

    /// Get path to calibration file
    fn get_calibration_path() -> PathBuf {
        let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
        PathBuf::from(home).join(".config/microduck/imu_calibration.bin")
    }

    /// Load calibration from file and apply to IMU
    /// Returns true if calibration was loaded, false if file doesn't exist
    pub fn load_calibration(&mut self) -> Result<bool> {
        let calib_path = Self::get_calibration_path();

        if !calib_path.exists() {
            return Ok(false);
        }

        // Read calibration data from file
        let calib_bytes = fs::read(&calib_path)
            .context(format!("Failed to read calibration file: {}", calib_path.display()))?;

        // Verify correct size (22 bytes for BNO055 calibration)
        if calib_bytes.len() != 22 {
            return Err(anyhow::anyhow!(
                "Invalid calibration file size: {} bytes (expected 22)",
                calib_bytes.len()
            ));
        }

        // Convert Vec to fixed-size array
        let mut buf = [0u8; 22];
        buf.copy_from_slice(&calib_bytes);

        // Convert bytes to BNO055Calibration
        let calibration = BNO055Calibration::from_buf(&buf);

        // Apply calibration to sensor
        self.imu.set_calibration_profile(calibration, &mut self.delay)
            .map_err(|e| anyhow::anyhow!("Failed to set calibration profile: {:?}", e))?;

        Ok(true)
    }

    /// Save current calibration to file
    pub fn save_calibration(&mut self) -> Result<()> {
        let calib_path = Self::get_calibration_path();

        // Create directory if it doesn't exist
        if let Some(parent) = calib_path.parent() {
            fs::create_dir_all(parent)
                .context("Failed to create calibration directory")?;
        }

        // Read calibration profile from sensor
        let calibration = self.imu.calibration_profile(&mut self.delay)
            .map_err(|e| anyhow::anyhow!("Failed to read calibration profile: {:?}", e))?;

        // Write to file
        let calib_bytes = calibration.as_bytes();
        fs::write(&calib_path, calib_bytes)
            .context(format!("Failed to write calibration file: {}", calib_path.display()))?;

        Ok(())
    }

    /// Set gravity calibration offset
    /// Call this after measuring gravity when robot is standing upright
    /// The offset will be subtracted from all future gravity readings
    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        self.gravity_offset = offset;
        println!("Set gravity offset: [{:.4}, {:.4}, {:.4}]", offset[0], offset[1], offset[2]);
    }

    /// Get current gravity offset
    pub fn get_gravity_offset(&self) -> [f64; 3] {
        self.gravity_offset
    }

    /// Rotate a vector by a quaternion
    /// Quaternion format: [w, x, y, z] (scalar-first convention)
    fn quat_rotate_vec(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
        let [w, qx, qy, qz] = quat;
        let [vx, vy, vz] = vec;

        // Compute quaternion-vector rotation: v' = q * v * q^(-1)
        // Using optimized formula: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + w * v)
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

    /// Rotate a vector by the inverse of a quaternion
    /// Quaternion format: [w, x, y, z] (scalar-first convention)
    /// This is used to transform from world frame to body frame
    /// Uses the formula: result = vec - w * t + xyz × t, where t = xyz × vec * 2
    fn quat_rotate_vec_inverse(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
        let [w, qx, qy, qz] = quat;
        let [vx, vy, vz] = vec;

        // t = xyz × vec * 2
        let tx = (qy * vz - qz * vy) * 2.0;
        let ty = (qz * vx - qx * vz) * 2.0;
        let tz = (qx * vy - qy * vx) * 2.0;

        // result = vec - w * t + xyz × t
        let cx = qy * tz - qz * ty;
        let cy = qz * tx - qx * tz;
        let cz = qx * ty - qy * tx;

        [
            vx - w * tx + cx,
            vy - w * ty + cy,
            vz - w * tz + cz,
        ]
    }

    /// Median of 3 values — used for single-sample spike rejection
    fn median3(a: f64, b: f64, c: f64) -> f64 {
        a.max(b).min(c).max(a.min(b))
    }

    /// Read current IMU data
    /// Returns gyroscope (rad/s) and normalized projected gravity (unit vector)
    /// Projected gravity can come from either:
    /// - Raw accelerometer (default): includes dynamic accelerations, impacts, vibrations
    /// - Quaternion-based (--projected-gravity): clean gravity free from dynamic accelerations
    pub fn read(&mut self) -> Result<ImuData> {
        // Read gyroscope (hardware-remapped, in 1/16 deg/s)
        let gyro = self.imu.gyro_data()
            .map_err(|e| anyhow::anyhow!("Failed to read gyroscope: {:?}", e))?;

        // Convert gyroscope to rad/s (from 1/16 deg/s)
        let gyro_scale = std::f64::consts::PI / 180.0;
        let gyro_raw = [
            gyro.x as f64 * gyro_scale,
            gyro.y as f64 * gyro_scale,
            gyro.z as f64 * gyro_scale,
        ];

        // 3-sample median filter: eliminates single-sample spikes with no tuning required
        let gyro_rad_s = [
            Self::median3(self.gyro_history[0][0], self.gyro_history[1][0], gyro_raw[0]),
            Self::median3(self.gyro_history[0][1], self.gyro_history[1][1], gyro_raw[1]),
            Self::median3(self.gyro_history[0][2], self.gyro_history[1][2], gyro_raw[2]),
        ];
        self.gyro_history[0] = self.gyro_history[1];
        self.gyro_history[1] = gyro_raw;

        // Compute projected gravity based on mode
        let proj_grav = if self.use_projected_gravity {
            // Mode 1: Quaternion-based projected gravity (clean, no dynamic accelerations)
            // Read quaternion from IMU's sensor fusion (IMU mode)
            let quat_mint = self.imu.quaternion()
                .map_err(|e| anyhow::anyhow!("Failed to read quaternion: {:?}", e))?;

            // Convert mint::Quaternion to [w, x, y, z] array
            let quat = [
                quat_mint.s as f64,    // w (scalar part)
                quat_mint.v.x as f64,  // x (vector part)
                quat_mint.v.y as f64,  // y
                quat_mint.v.z as f64,  // z
            ];

            // Compute projected gravity by rotating world-frame gravity into body frame
            // World gravity: [0, 0, -1.0] unit vector pointing downward
            // Use inverse rotation to transform from world frame to body frame
            let world_gravity = [0.0, 0.0, -1.0];
            let proj_grav_unit = Self::quat_rotate_vec_inverse(quat, world_gravity);

            // Already a unit vector (or very close), but normalize to be safe
            let mag = (proj_grav_unit[0].powi(2) + proj_grav_unit[1].powi(2) + proj_grav_unit[2].powi(2)).sqrt();
            if mag > 0.1 {
                [
                    proj_grav_unit[0] / mag,
                    proj_grav_unit[1] / mag,
                    proj_grav_unit[2] / mag,
                ]
            } else {
                [0.0, 0.0, -1.0] // Default to downward unit vector if magnitude is too small
            }
        } else {
            // Mode 2: Raw accelerometer (default, includes dynamic accelerations)
            // Read raw accelerometer (hardware-remapped, in m/s²)
            let accel = self.imu.accel_data()
                .map_err(|e| anyhow::anyhow!("Failed to read accelerometer: {:?}", e))?;

            // Convert accelerometer to f64 (already in m/s²)
            let accel_ms2 = [
                accel.x as f64,
                accel.y as f64,
                accel.z as f64,
            ];

            // Accelerometer measures normal force (pointing up when at rest)
            // Negate to get gravity direction (pointing down)
            let accel_negated = [
                -accel_ms2[0],
                -accel_ms2[1],
                -accel_ms2[2],
            ];

            // Normalize to unit vector
            let mag = (accel_negated[0].powi(2) + accel_negated[1].powi(2) + accel_negated[2].powi(2)).sqrt();
            if mag > 0.1 {
                [
                    accel_negated[0] / mag,
                    accel_negated[1] / mag,
                    accel_negated[2] / mag,
                ]
            } else {
                [0.0, 0.0, -1.0] // Default to downward unit vector if magnitude is too small
            }
        };

        // Apply calibration offset (in unit vector space)
        let accel_corrected = [
            proj_grav[0] - self.gravity_offset[0],
            proj_grav[1] - self.gravity_offset[1],
            proj_grav[2] - self.gravity_offset[2],
        ];

        // Renormalize after offset
        let mag2 = (accel_corrected[0].powi(2) + accel_corrected[1].powi(2) + accel_corrected[2].powi(2)).sqrt();
        let accel_final = if mag2 > 0.1 {
            [
                accel_corrected[0] / mag2,
                accel_corrected[1] / mag2,
                accel_corrected[2] / mag2,
            ]
        } else {
            [0.0, 0.0, -1.0]
        };

        // 3-sample median filter on accel/projected gravity
        let accel_filtered = [
            Self::median3(self.accel_history[0][0], self.accel_history[1][0], accel_final[0]),
            Self::median3(self.accel_history[0][1], self.accel_history[1][1], accel_final[1]),
            Self::median3(self.accel_history[0][2], self.accel_history[1][2], accel_final[2]),
        ];
        self.accel_history[0] = self.accel_history[1];
        self.accel_history[1] = accel_final;

        Ok(ImuData {
            gyro: gyro_rad_s,
            accel: accel_filtered,
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
