use anyhow::{Context, Result};
use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::os::unix::io::AsRawFd;
use std::thread;
use std::time::Duration;

// BNO055 Register addresses
const BNO055_CHIP_ID: u8 = 0x00;
const BNO055_PAGE_ID: u8 = 0x07;
const BNO055_OPR_MODE: u8 = 0x3D;
const BNO055_PWR_MODE: u8 = 0x3E;
const BNO055_SYS_TRIGGER: u8 = 0x3F;
const BNO055_CALIB_STAT: u8 = 0x35;

// Quaternion data registers
const BNO055_QUA_DATA_W_LSB: u8 = 0x20;

// Gyroscope data registers
const BNO055_GYR_DATA_X_LSB: u8 = 0x14;

// Operation modes
const BNO055_MODE_CONFIG: u8 = 0x00;
const BNO055_MODE_NDOF: u8 = 0x0C;

// Power modes
const BNO055_POWER_MODE_NORMAL: u8 = 0x00;

// I2C constants
const I2C_SLAVE: u16 = 0x0703;

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
    i2c: std::fs::File,
}

impl ImuController {
    /// Create a new IMU controller
    /// i2c_device: path to I2C device (e.g., "/dev/i2c-1")
    /// address: BNO055 I2C address (0x28 or 0x29)
    pub fn new(i2c_device: &str, address: u8) -> Result<Self> {
        let i2c = OpenOptions::new()
            .read(true)
            .write(true)
            .open(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;

        // Set I2C slave address
        unsafe {
            if libc::ioctl(
                i2c.as_raw_fd(),
                I2C_SLAVE as libc::c_ulong,
                address as libc::c_ulong,
            ) < 0
            {
                return Err(anyhow::anyhow!("Failed to set I2C slave address to 0x{:02X}", address));
            }
        }

        let mut controller = Self { i2c };

        // Verify chip ID
        let chip_id = controller.read_register(BNO055_CHIP_ID)?;
        if chip_id != 0xA0 {
            return Err(anyhow::anyhow!(
                "Invalid chip ID: 0x{:02X}, expected 0xA0",
                chip_id
            ));
        }

        // Reset
        controller.write_register(BNO055_SYS_TRIGGER, 0x20)?;
        thread::sleep(Duration::from_millis(650)); // Wait for reset

        // Set to normal power mode
        controller.write_register(BNO055_PWR_MODE, BNO055_POWER_MODE_NORMAL)?;
        thread::sleep(Duration::from_millis(10));

        // Select page 0
        controller.write_register(BNO055_PAGE_ID, 0x00)?;
        thread::sleep(Duration::from_millis(10));

        // Set to NDOF mode (9-axis fusion)
        controller.write_register(BNO055_OPR_MODE, BNO055_MODE_NDOF)?;
        thread::sleep(Duration::from_millis(100));

        Ok(controller)
    }

    /// Create a new IMU controller with default settings
    /// Uses /dev/i2c-1 and address 0x28
    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", 0x28)
    }

    /// Read a single register
    fn read_register(&mut self, reg: u8) -> Result<u8> {
        self.i2c.write(&[reg])?;
        thread::sleep(Duration::from_micros(100));
        let mut buffer = [0u8; 1];
        self.i2c.read(&mut buffer)?;
        Ok(buffer[0])
    }

    /// Write a single register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<()> {
        self.i2c.write(&[reg, value])?;
        Ok(())
    }

    /// Read multiple bytes starting from a register
    fn read_bytes(&mut self, reg: u8, buffer: &mut [u8]) -> Result<()> {
        self.i2c.write(&[reg])?;
        thread::sleep(Duration::from_micros(100));
        self.i2c.read(buffer)?;
        Ok(())
    }

    /// Read raw accelerometer data in sensor frame (m/s²)
    /// Returns [sensor_x, sensor_y, sensor_z]
    pub fn read_raw_accelerometer(&mut self) -> Result<[f64; 3]> {
        const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
        let mut accel_buffer = [0u8; 6];
        self.read_bytes(BNO055_ACC_DATA_X_LSB, &mut accel_buffer)?;

        // BNO055 accelerometer scale: 1 LSB = 0.01 m/s² (in m/s² mode, which is default in NDOF)
        Ok([
            i16::from_le_bytes([accel_buffer[0], accel_buffer[1]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buffer[2], accel_buffer[3]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buffer[4], accel_buffer[5]]) as f64 / 100.0,
        ])
    }

    /// Read current IMU data
    pub fn read(&mut self) -> Result<ImuData> {
        // Read accelerometer data (6 bytes: X, Y, Z as 16-bit signed integers)
        // This measures specific force (proper acceleration), which includes gravity + any contact forces
        const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
        let mut accel_buffer = [0u8; 6];
        self.read_bytes(BNO055_ACC_DATA_X_LSB, &mut accel_buffer)?;

        // BNO055 accelerometer scale: 1 LSB = 0.01 m/s² (in m/s² mode, which is default in NDOF)
        let accel_sensor = [
            i16::from_le_bytes([accel_buffer[0], accel_buffer[1]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buffer[2], accel_buffer[3]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buffer[4], accel_buffer[5]]) as f64 / 100.0,
        ];

        // Read gyroscope data (6 bytes: X, Y, Z as 16-bit signed integers)
        let mut gyro_buffer = [0u8; 6];
        self.read_bytes(BNO055_GYR_DATA_X_LSB, &mut gyro_buffer)?;

        // Convert to rad/s (BNO055 gyro scale: 1 LSB = 1/16 dps = 1/16 * π/180 rad/s)
        let scale = 1.0 / 16.0 * std::f64::consts::PI / 180.0;
        let gyro_sensor = [
            i16::from_le_bytes([gyro_buffer[0], gyro_buffer[1]]) as f64 * scale,  // sensor X
            i16::from_le_bytes([gyro_buffer[2], gyro_buffer[3]]) as f64 * scale,  // sensor Y
            i16::from_le_bytes([gyro_buffer[4], gyro_buffer[5]]) as f64 * scale,  // sensor Z
        ];

        // Transform gyroscope to robot frame
        // Empirically determined from sensor measurements:
        // - Sensor Y → Robot X (forward)
        // - Sensor X (right) → Robot Y (left, negated)
        // - Sensor Z (up) → Robot Z (up, no negation for gyro)
        let gyro = [
            gyro_sensor[1],    // robot X = sensor Y
            -gyro_sensor[0],   // robot Y = -sensor X
            gyro_sensor[2],    // robot Z = sensor Z (CCW rotation gives positive)
        ];

        // Transform accelerometer to get projected gravity
        // Note: Z axis handling differs from gyro due to gravity direction
        let accel_raw = [
            accel_sensor[1],    // proj grav X = sensor Y
            -accel_sensor[0],   // proj grav Y = -sensor X
            -accel_sensor[2],   // proj grav Z = -sensor Z (gravity points down)
        ];

        // Normalize to unit length (MuJoCo uses normalized projected gravity)
        let mag = (accel_raw[0].powi(2) + accel_raw[1].powi(2) + accel_raw[2].powi(2)).sqrt();
        let accel = if mag > 0.1 {
            [
                accel_raw[0] / mag,
                accel_raw[1] / mag,
                accel_raw[2] / mag,
            ]
        } else {
            [0.0, 0.0, -1.0] // Default to downward unit vector if magnitude is too small
        };

        Ok(ImuData {
            gyro,
            accel,
        })
    }

    /// Get calibration status
    /// Returns (system, gyro, accel, mag) calibration levels (0-3)
    pub fn get_calibration_status(&mut self) -> Result<(u8, u8, u8, u8)> {
        let status = self.read_register(BNO055_CALIB_STAT)?;

        // Extract calibration status for each sensor
        let mag = (status >> 0) & 0x03;
        let acc = (status >> 2) & 0x03;
        let gyr = (status >> 4) & 0x03;
        let sys = (status >> 6) & 0x03;

        Ok((sys, gyr, acc, mag))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_data_default() {
        let data = ImuData::default();

        // Check default normalized projected gravity (upright robot, unit vector pointing down)
        assert_eq!(data.accel[2], -1.0);
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
        // Test that default ImuData has downward-pointing normalized projected gravity
        let data = ImuData::default();

        // When upright at rest, gravity points down as unit vector
        assert!((data.accel[0] - 0.0).abs() < 1e-10);
        assert!((data.accel[1] - 0.0).abs() < 1e-10);
        assert!((data.accel[2] - (-1.0)).abs() < 1e-10);

        // Verify it's normalized (magnitude = 1)
        let mag = (data.accel[0].powi(2) + data.accel[1].powi(2) + data.accel[2].powi(2)).sqrt();
        assert!((mag - 1.0).abs() < 1e-10);
    }
}
