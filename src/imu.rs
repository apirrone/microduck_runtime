use anyhow::{Context, Result};
use linux_embedded_hal::{Delay, I2cdev};
use bno055::{Bno055, AxisRemap, BNO055AxisConfig, BNO055AxisSign, BNO055Calibration};
use std::fs;
use std::path::PathBuf;
use std::io::{Read as IoRead, Write as IoWrite};
use std::os::unix::io::AsRawFd;
use std::thread;
use std::time::Duration;

/// IMU data containing gyroscope and normalized projected gravity
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s (angular velocity in body frame)
    pub gyro: [f64; 3],
    /// Normalized projected gravity [x, y, z] (unit vector in body frame)
    /// Computed by rotating world gravity [0, 0, -9.81] into body frame using
    /// the IMU's orientation estimate (quaternion from NDOF sensor fusion).
    /// This gives clean gravity free from dynamic accelerations and noise.
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
        // IMU mounting: X+ right, Y+ forward, Z+ up
        // Robot frame: X+ forward, Y+ left, Z+ up
        // Mapping: Robot = [+Sensor_Y, -Sensor_X, +Sensor_Z]
        let remap = AxisRemap::builder()
            .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)  // Swap X and Y axes
            .build()
            .map_err(|_| anyhow::anyhow!("Failed to build axis remap"))?;

        imu.set_axis_remap(remap)
            .map_err(|e| anyhow::anyhow!("Failed to set axis remap: {:?}", e))?;

        // Flip only Y axis sign (sensor's right becomes robot's left)
        imu.set_axis_sign(BNO055AxisSign::Y_NEGATIVE)
            .map_err(|e| anyhow::anyhow!("Failed to set axis sign: {:?}", e))?;

        // Set to NDOF mode
        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .map_err(|e| anyhow::anyhow!("Failed to set NDOF mode: {:?}", e))?;

        let mut controller = Self { imu, delay };

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

    /// Rotate a vector by a quaternion
    /// Quaternion format: [w, x, y, z]
    fn quat_rotate_vec(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
        let [w, qx, qy, qz] = quat;
        let [vx, vy, vz] = vec;

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

    /// Read quaternion directly from BNO055 via I2C
    /// Returns quaternion in [w, x, y, z] format
    fn read_quaternion(&mut self) -> Result<[f64; 4]> {
        use std::fs::OpenOptions;

        // Open I2C device
        let i2c_device = "/dev/i2c-1";
        let address = 0x29u8; // Alternative address
        let mut i2c = OpenOptions::new()
            .read(true)
            .write(true)
            .open(i2c_device)
            .context("Failed to open I2C device for quaternion reading")?;

        // Set I2C slave address
        const I2C_SLAVE: u16 = 0x0703;
        unsafe {
            if libc::ioctl(
                i2c.as_raw_fd(),
                I2C_SLAVE as libc::c_ulong,
                address as libc::c_ulong,
            ) < 0
            {
                return Err(anyhow::anyhow!("Failed to set I2C slave address"));
            }
        }

        // Read quaternion data
        const BNO055_QUA_DATA_W_LSB: u8 = 0x20;
        let mut quat_buffer = [0u8; 8];
        i2c.write(&[BNO055_QUA_DATA_W_LSB])
            .context("Failed to write quaternion register address")?;
        thread::sleep(Duration::from_micros(100));
        i2c.read(&mut quat_buffer)
            .context("Failed to read quaternion data")?;

        // Convert to quaternion (BNO055 scale: 1 LSB = 1/16384)
        let scale = 1.0 / 16384.0;
        let quat = [
            i16::from_le_bytes([quat_buffer[0], quat_buffer[1]]) as f64 * scale, // w
            i16::from_le_bytes([quat_buffer[2], quat_buffer[3]]) as f64 * scale, // x
            i16::from_le_bytes([quat_buffer[4], quat_buffer[5]]) as f64 * scale, // y
            i16::from_le_bytes([quat_buffer[6], quat_buffer[7]]) as f64 * scale, // z
        ];

        Ok(quat)
    }

    /// Read current IMU data
    /// Returns gyroscope (rad/s) and normalized projected gravity (unit vector)
    /// Projected gravity is computed by rotating world-frame gravity [0, 0, -9.81]
    /// into body frame using the IMU's orientation estimate (quaternion from sensor fusion)
    pub fn read(&mut self) -> Result<ImuData> {
        // Read gyroscope (hardware-remapped, in 1/16 deg/s)
        let gyro = self.imu.gyro_data()
            .map_err(|e| anyhow::anyhow!("Failed to read gyroscope: {:?}", e))?;

        // Convert gyroscope to rad/s (from 1/16 deg/s)
        let gyro_scale = std::f64::consts::PI / 180.0;
        let gyro_rad_s = [
            gyro.x as f64 * gyro_scale,
            gyro.y as f64 * gyro_scale,
            gyro.z as f64 * gyro_scale,
        ];

        // Read quaternion from IMU's sensor fusion
        let quat = self.read_quaternion()
            .context("Failed to read quaternion")?;

        // Compute projected gravity by rotating world-frame gravity into body frame
        // World gravity: [0, 0, -9.81] pointing downward
        // This matches how MuJoCo/simulation computes projected gravity
        let world_gravity = [0.0, 0.0, -9.81];
        let proj_grav_ms2 = Self::quat_rotate_vec(quat, world_gravity);

        // Normalize to unit vector (MuJoCo expects normalized projected gravity)
        let mag = (proj_grav_ms2[0].powi(2) + proj_grav_ms2[1].powi(2) + proj_grav_ms2[2].powi(2)).sqrt();
        let proj_grav = if mag > 0.1 {
            [
                proj_grav_ms2[0] / mag,
                proj_grav_ms2[1] / mag,
                proj_grav_ms2[2] / mag,
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
