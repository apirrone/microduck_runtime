use anyhow::{Context, Result};
use linux_embedded_hal::{Delay, I2cdev};
use embedded_hal::i2c::I2c;
use bno055::{Bno055, BNO055AxisSign, BNO055Calibration};
use std::fs;
use std::path::PathBuf;
use std::thread;
use std::time::Duration;

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

// --- Shared utility functions ---

/// Median of 3 values — used for single-sample spike rejection
fn median3(a: f64, b: f64, c: f64) -> f64 {
    a.max(b).min(c).max(a.min(b))
}

/// Rotate a vector by the inverse of a quaternion
/// Quaternion format: [w, x, y, z] (scalar-first convention)
/// Used to transform from world frame to body frame
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

/// Normalize and apply calibration offset to a gravity vector, returning a unit vector
fn finalize_gravity(raw: [f64; 3], offset: [f64; 3]) -> [f64; 3] {
    let mag = (raw[0].powi(2) + raw[1].powi(2) + raw[2].powi(2)).sqrt();
    let normalized = if mag > 0.1 {
        [raw[0] / mag, raw[1] / mag, raw[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    };

    let corrected = [
        normalized[0] - offset[0],
        normalized[1] - offset[1],
        normalized[2] - offset[2],
    ];

    let mag2 = (corrected[0].powi(2) + corrected[1].powi(2) + corrected[2].powi(2)).sqrt();
    if mag2 > 0.1 {
        [corrected[0] / mag2, corrected[1] / mag2, corrected[2] / mag2]
    } else {
        [0.0, 0.0, -1.0]
    }
}

// --- BNO055 Controller ---

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
            median3(self.gyro_history[0][0], self.gyro_history[1][0], gyro_raw[0]),
            median3(self.gyro_history[0][1], self.gyro_history[1][1], gyro_raw[1]),
            median3(self.gyro_history[0][2], self.gyro_history[1][2], gyro_raw[2]),
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
            let proj_grav_unit = quat_rotate_vec_inverse(quat, world_gravity);

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
            median3(self.accel_history[0][0], self.accel_history[1][0], accel_final[0]),
            median3(self.accel_history[0][1], self.accel_history[1][1], accel_final[1]),
            median3(self.accel_history[0][2], self.accel_history[1][2], accel_final[2]),
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

// --- BNO08X Controller ---

// SHTP (SensorHub Transport Protocol) constants for BNO08X
const BNO08X_CHANNEL_EXECUTABLE: u8 = 1;
const BNO08X_CHANNEL_CONTROL: u8 = 2;
const BNO08X_CHANNEL_REPORTS: u8 = 3;
const BNO08X_REPORT_ROTATION_VECTOR: u8 = 0x08;  // Game Rotation Vector: gyro + accel only (no magnetometer)
const BNO08X_REPORT_GYRO_CALIBRATED: u8 = 0x02;
const BNO08X_CMD_SET_FEATURE: u8 = 0xFD;

/// IMU controller for BNO08X sensor family (BNO080/085/086) using a custom SHTP driver.
///
/// Talks directly to the sensor over I2C using the SHTP (SensorHub Transport Protocol).
/// NACK responses (sensor has no data ready) are handled gracefully by returning cached values,
/// rather than erroring out or freezing.
pub struct Bno08xController {
    dev: I2cdev,
    addr: u8,
    send_seq: [u8; 8],         // per-channel sequence numbers for outbound packets
    last_gyro: [f64; 3],       // last received gyro in sensor frame (rad/s)
    last_quat: [f64; 4],       // last received quaternion [w, x, y, z] in sensor frame
    gravity_offset: [f64; 3],
    gyro_history: [[f64; 3]; 2],
    accel_history: [[f64; 3]; 2],
    needs_reconfigure: bool,   // set when a sensor reset is detected (ch=0 advertisement seen)
}

impl Bno08xController {
    /// Default I2C address (SA0=GND → 0x4A)
    pub const DEFAULT_ADDRESS: u8 = 0x4A;
    /// Alternate I2C address (SA0=VCC → 0x4B)
    pub const ALTERNATE_ADDRESS: u8 = 0x4B;

    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", Self::ALTERNATE_ADDRESS)
    }

    pub fn new(i2c_device: &str, addr: u8) -> Result<Self> {
        let dev = I2cdev::new(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;

        let mut ctrl = Self {
            dev,
            addr,
            send_seq: [0u8; 8],
            last_gyro: [0.0; 3],
            last_quat: [0.0, 0.0, 0.0, 1.0],  // identity quaternion
            gravity_offset: [0.0; 3],
            gyro_history: [[0.0; 3]; 2],
            accel_history: [[0.0; 3]; 2],
            needs_reconfigure: false,
        };

        ctrl.shtp_init()
            .context("Failed to initialize BNO08X over SHTP")?;

        Ok(ctrl)
    }

    /// Enable Game Rotation Vector and Gyro Calibrated reports at the given period.
    fn configure_reports(&mut self, period_us: u32) -> Result<()> {
        let p = period_us.to_le_bytes();
        self.write_packet(BNO08X_CHANNEL_CONTROL, &[
            BNO08X_CMD_SET_FEATURE, BNO08X_REPORT_ROTATION_VECTOR,
            0, 0, 0,
            p[0], p[1], p[2], p[3],
            0, 0, 0, 0,
            0, 0, 0, 0,
        ])?;
        thread::sleep(Duration::from_millis(10));
        self.write_packet(BNO08X_CHANNEL_CONTROL, &[
            BNO08X_CMD_SET_FEATURE, BNO08X_REPORT_GYRO_CALIBRATED,
            0, 0, 0,
            p[0], p[1], p[2], p[3],
            0, 0, 0, 0,
            0, 0, 0, 0,
        ])?;
        Ok(())
    }

    /// Initialise the BNO08X: soft-reset, drain startup packets, enable reports.
    fn shtp_init(&mut self) -> Result<()> {
        // Soft reset
        self.write_packet(BNO08X_CHANNEL_EXECUTABLE, &[0x01])?;
        thread::sleep(Duration::from_millis(300));

        // Drain all advertisement / product-ID packets the sensor sends on boot
        for _ in 0..20 {
            let _ = self.read_shtp_packet();
            thread::sleep(Duration::from_millis(5));
        }

        // Enable rotation vector and gyro at 5 ms period (200 Hz)
        self.configure_reports(5_000)?;
        thread::sleep(Duration::from_millis(50));

        // Prime internal state with a few real readings
        for _ in 0..40 {
            self.drain_once();
            thread::sleep(Duration::from_millis(5));
        }

        // Clear any reset flag that may have been set during the init drain
        self.needs_reconfigure = false;

        Ok(())
    }

    /// Send one SHTP packet on the given channel.
    fn write_packet(&mut self, channel: u8, payload: &[u8]) -> Result<()> {
        let total_len = (payload.len() + 4) as u16;
        let seq = self.send_seq[channel as usize];
        self.send_seq[channel as usize] = seq.wrapping_add(1);

        let mut buf = Vec::with_capacity(total_len as usize);
        buf.push(total_len as u8);
        buf.push((total_len >> 8) as u8);
        buf.push(channel);
        buf.push(seq);
        buf.extend_from_slice(payload);

        self.dev.write(self.addr, &buf)
            .map_err(|e| anyhow::anyhow!("BNO08X I2C write failed: {:?}", e))?;
        Ok(())
    }

    /// Try to read one SHTP packet from the sensor.
    ///
    /// Returns `None` when the sensor NACKs (no data ready).  This is the normal
    /// "no new data" case and must NOT be treated as an error.
    ///
    /// Uses a single 300-byte I2C read.  Per the SHTP spec, the BNO08X fills
    /// any bytes beyond the actual packet length with 0xFF, so over-reading is
    /// safe.  Using a single transaction is required because the sensor advances
    /// its read pointer on every I2C STOP condition — a separate header-only read
    /// would consume the packet before we can read its payload.
    fn read_shtp_packet(&mut self) -> Option<(u8, Vec<u8>)> {
        let mut buf = [0u8; 300];
        if self.dev.read(self.addr, &mut buf).is_err() {
            // Either NACK (no data ready) or EIO (clock-stretch timeout from BCM2835 I2C).
            // Both are transient — treat as "no data, try again".
            return None;
        }

        // Bit 15 of length MSB is the "continuation" flag for multi-packet msgs.
        let total_len = ((buf[1] as usize & 0x7F) << 8) | buf[0] as usize;
        let channel = buf[2];

        if total_len < 4 {
            return None;  // guard against corrupted header
        }
        if total_len == 4 {
            return Some((channel, vec![]));  // header-only packet (valid)
        }

        let payload_end = total_len.min(buf.len());
        let payload = buf[4..payload_end].to_vec();
        Some((channel, payload))
    }

    /// Read one packet and update internal gyro/quat caches if it's a sensor report.
    /// Returns `true` if a packet was received (regardless of report type).
    fn drain_once(&mut self) -> bool {
        let (channel, payload) = match self.read_shtp_packet() {
            Some(p) => p,
            None => return false,
        };

        if channel == 0 {
            // SHTP advertisement = sensor just reset. Schedule re-configuration.
            self.needs_reconfigure = true;
            return true;
        }

        if channel != BNO08X_CHANNEL_REPORTS {
            return true;  // control/executable packet — not a sensor report
        }

        if payload.is_empty() {
            return true;
        }

        // Channel 3 payload layout:
        //   One or more leading timestamp records: 0xFB (Timestamp Rebase) or
        //   0xFA (Base Timestamp Reference), each exactly 5 bytes (type + 4-byte value).
        //   Followed by back-to-back sensor reports, each:
        //     [report_id(1), seq(1), status(1), delay(1), data...]
        //   Report sizes (total bytes including header):
        //     0x02 Gyro Calibrated:      10  (4 hdr + 3×i16 Q9)
        //     0x08 Game Rotation Vector: 12  (4 hdr + 4×i16 Q14)

        // Skip all leading timestamp records.
        let mut cursor = 0;
        while cursor + 5 <= payload.len() {
            match payload[cursor] {
                0xFB | 0xFA => cursor += 5,
                _ => break,
            }
        }

        // Iterate through all sensor reports in this packet.
        while cursor < payload.len() {
            let report_id = payload[cursor];
            match report_id {
                BNO08X_REPORT_GYRO_CALIBRATED => {
                    if cursor + 10 > payload.len() { break; }
                    let x = i16::from_le_bytes([payload[cursor+4], payload[cursor+5]]) as f64 / 512.0;
                    let y = i16::from_le_bytes([payload[cursor+6], payload[cursor+7]]) as f64 / 512.0;
                    let z = i16::from_le_bytes([payload[cursor+8], payload[cursor+9]]) as f64 / 512.0;
                    self.last_gyro = [x, y, z];
                    cursor += 10;
                }
                BNO08X_REPORT_ROTATION_VECTOR => {  // = 0x08 Game Rotation Vector
                    if cursor + 12 > payload.len() { break; }
                    let qi = i16::from_le_bytes([payload[cursor+4],  payload[cursor+5]])  as f64 / 16384.0;
                    let qj = i16::from_le_bytes([payload[cursor+6],  payload[cursor+7]])  as f64 / 16384.0;
                    let qk = i16::from_le_bytes([payload[cursor+8],  payload[cursor+9]])  as f64 / 16384.0;
                    let qw = i16::from_le_bytes([payload[cursor+10], payload[cursor+11]]) as f64 / 16384.0;
                    self.last_quat = [qw, qi, qj, qk];  // [w, x, y, z]
                    cursor += 12;
                }
                _ => break,  // unknown report ID — can't determine its size, stop here
            }
        }

        true
    }

    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        self.gravity_offset = offset;
        println!("Set gravity offset: [{:.4}, {:.4}, {:.4}]", offset[0], offset[1], offset[2]);
    }

    pub fn get_gravity_offset(&self) -> [f64; 3] {
        self.gravity_offset
    }

    pub fn read(&mut self) -> Result<ImuData> {
        // Drain whatever is buffered right now (bounded to avoid spinning on reset loops).
        // This also sets needs_reconfigure if we see ch=0 advertisement packets.
        for _ in 0..10 { if !self.drain_once() { break; } }

        // If the sensor reset, re-configure it before reading.
        if self.needs_reconfigure {
            self.needs_reconfigure = false;
            let _ = self.configure_reports(5_000);
            thread::sleep(Duration::from_millis(100));  // give sensor time to start reporting
            for _ in 0..10 { if !self.drain_once() { break; } }
        }

        // Spin-poll until we get a sensor report (or timeout after 100ms).
        // No sleep between attempts: each EIO takes only ~640µs (BCM2835 clock-stretch
        // timeout), so spinning gives ~1500 retries/second to catch the brief window
        // when the sensor's data is fully ready and it won't clock-stretch.
        let deadline = std::time::Instant::now() + Duration::from_millis(100);
        loop {
            if self.drain_once() {
                for _ in 0..10 { if !self.drain_once() { break; } }
                break;
            }
            if std::time::Instant::now() >= deadline {
                break;  // sensor not responding; return cached values
            }
        }

        // Apply axis remap: Robot = [-Sensor_X, -Sensor_Y, +Sensor_Z]
        let gyro_raw = [
            -self.last_gyro[0],
            -self.last_gyro[1],
             self.last_gyro[2],
        ];

        // 3-sample median filter (spike rejection)
        let gyro_filtered = [
            median3(self.gyro_history[0][0], self.gyro_history[1][0], gyro_raw[0]),
            median3(self.gyro_history[0][1], self.gyro_history[1][1], gyro_raw[1]),
            median3(self.gyro_history[0][2], self.gyro_history[1][2], gyro_raw[2]),
        ];
        self.gyro_history[0] = self.gyro_history[1];
        self.gyro_history[1] = gyro_raw;

        // Compute projected gravity from rotation quaternion, then remap axes
        let world_gravity = [0.0, 0.0, -1.0];
        let grav_sensor = quat_rotate_vec_inverse(self.last_quat, world_gravity);
        let grav_robot = [
            -grav_sensor[0],
            -grav_sensor[1],
             grav_sensor[2],
        ];

        let accel_final = finalize_gravity(grav_robot, self.gravity_offset);

        let accel_filtered = [
            median3(self.accel_history[0][0], self.accel_history[1][0], accel_final[0]),
            median3(self.accel_history[0][1], self.accel_history[1][1], accel_final[1]),
            median3(self.accel_history[0][2], self.accel_history[1][2], accel_final[2]),
        ];
        self.accel_history[0] = self.accel_history[1];
        self.accel_history[1] = accel_final;

        Ok(ImuData {
            gyro: gyro_filtered,
            accel: accel_filtered,
        })
    }
}

// --- Unified IMU controller ---

/// Wraps either a BNO055 or BNO08X controller behind a common interface
pub enum AnyImuController {
    Bno055(ImuController),
    Bno08x(Bno08xController),
}

impl AnyImuController {
    pub fn read(&mut self) -> Result<ImuData> {
        match self {
            AnyImuController::Bno055(c) => c.read(),
            AnyImuController::Bno08x(c) => c.read(),
        }
    }

    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        match self {
            AnyImuController::Bno055(c) => c.set_gravity_offset(offset),
            AnyImuController::Bno08x(c) => c.set_gravity_offset(offset),
        }
    }

    pub fn get_gravity_offset(&self) -> [f64; 3] {
        match self {
            AnyImuController::Bno055(c) => c.get_gravity_offset(),
            AnyImuController::Bno08x(c) => c.get_gravity_offset(),
        }
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
