use anyhow::{Context, Result};
use linux_embedded_hal::{Delay, I2cdev};
use embedded_hal::i2c::I2c;
use bno055::{Bno055, BNO055AxisSign, BNO055Calibration};
use bmi088::{Bmi088, Bmi088Ahrs, Config as Bmi088Config};
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

/// IMU data containing gyroscope, normalized projected gravity, and orientation quaternion
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
    /// Orientation quaternion [w, x, y, z] (body→world, scalar-first convention)
    /// Represents the orientation of trunk_base in the world frame.
    /// Defaults to identity (upright, no rotation).
    pub quat: [f64; 4],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, -1.0], // Default normalized projected gravity (upright robot, unit vector pointing down)
            quat: [1.0, 0.0, 0.0, 0.0], // Identity quaternion (no rotation)
        }
    }
}

// --- Shared utility functions ---

/// Median of 3 values — used for single-sample spike rejection
fn median3(a: f64, b: f64, c: f64) -> f64 {
    a.max(b).min(c).max(a.min(b))
}

/// Rotate a vector by the inverse of a quaternion: q⁻¹ * v * q
/// Quaternion format: [w, x, y, z] (scalar-first convention)
/// Used by BNO055 (body→world convention: apply inverse to get world→body)
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

/// Rotate a vector directly by a quaternion: q * v * q⁻¹
/// Quaternion format: [w, x, y, z] (scalar-first convention)
/// Used by BNO08X (reference→body convention: apply directly to get world→body)
fn quat_rotate_vec(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
    let [w, qx, qy, qz] = quat;
    let [vx, vy, vz] = vec;

    let tx = (qy * vz - qz * vy) * 2.0;
    let ty = (qz * vx - qx * vz) * 2.0;
    let tz = (qx * vy - qy * vx) * 2.0;

    let cx = qy * tz - qz * ty;
    let cy = qz * tx - qx * tz;
    let cz = qx * ty - qy * tx;

    [
        vx + w * tx + cx,
        vy + w * ty + cy,
        vz + w * tz + cz,
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
    /// World-frame gravity unit vector used for quaternion-based projected gravity.
    /// Standard robot (z+=up): [0, 0, -1]. Fold robot (x+=up): [-1, 0, 0].
    world_gravity: [f64; 3],
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
            world_gravity: [0.0, 0.0, -1.0],
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

    /// Set the world-frame gravity unit vector used for quaternion-based projected gravity.
    /// Default: [0, 0, -1] (z+=up, standard microduck). Fold robot: [-1, 0, 0] (x+=up).
    pub fn set_world_gravity(&mut self, g: [f64; 3]) {
        self.world_gravity = g;
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

        // Always read quaternion for orientation tracking (used by streaming / digital twin)
        let quat_mint = self.imu.quaternion()
            .map_err(|e| anyhow::anyhow!("Failed to read quaternion: {:?}", e))?;
        let quat = [
            quat_mint.s as f64,    // w (scalar part)
            quat_mint.v.x as f64,  // x (vector part)
            quat_mint.v.y as f64,  // y
            quat_mint.v.z as f64,  // z
        ];

        // Compute projected gravity based on mode
        let proj_grav = if self.use_projected_gravity {
            // Mode 1: Quaternion-based projected gravity (clean, no dynamic accelerations)
            // Compute projected gravity by rotating world-frame gravity into body frame
            // World gravity: [0, 0, -1.0] unit vector pointing downward
            // Use inverse rotation to transform from world frame to body frame
            let proj_grav_unit = quat_rotate_vec_inverse(quat, self.world_gravity);

            // Already a unit vector (or very close), but normalize to be safe
            let mag = (proj_grav_unit[0].powi(2) + proj_grav_unit[1].powi(2) + proj_grav_unit[2].powi(2)).sqrt();
            if mag > 0.1 {
                [
                    proj_grav_unit[0] / mag,
                    proj_grav_unit[1] / mag,
                    proj_grav_unit[2] / mag,
                ]
            } else {
                self.world_gravity // Fall back to world gravity direction if magnitude too small
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
            quat,
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

/// Sensor state shared between the background poll thread and the main read() call.
struct Bno08xState {
    last_gyro: [f64; 3],
    last_quat: [f64; 4],
}

/// Owns the I2C handle and all SHTP protocol logic. Lives exclusively in the poll thread.
struct ShtpDevice {
    dev: I2cdev,
    addr: u8,
    send_seq: [u8; 8],
}

impl ShtpDevice {
    fn new(i2c_device: &str, addr: u8) -> Result<Self> {
        let dev = I2cdev::new(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;
        Ok(Self { dev, addr, send_seq: [0u8; 8] })
    }

    fn read_packet(&mut self) -> Option<(u8, Vec<u8>)> {
        let mut buf = [0u8; 128];
        if self.dev.read(self.addr, &mut buf).is_err() {
            return None;  // NACK or EIO — transient, caller retries
        }
        let total_len = ((buf[1] as usize & 0x7F) << 8) | buf[0] as usize;
        let channel = buf[2];
        if total_len < 4 { return None; }
        if total_len == 4 { return Some((channel, vec![])); }
        let payload_end = total_len.min(buf.len());
        Some((channel, buf[4..payload_end].to_vec()))
    }

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
            .map_err(|e| anyhow::anyhow!("BNO08X I2C write failed: {:?}", e))
    }

    fn configure_reports(&mut self, gyro_period_us: u32) -> Result<()> {
        // Rotation vector runs at 2× the gyro period (half rate) to reduce the
        // frequency of BCM2835 clock-stretch EIO caused by sensor fusion computation.
        let quat_period_us = gyro_period_us * 2;
        let g = gyro_period_us.to_le_bytes();
        let q = quat_period_us.to_le_bytes();
        self.write_packet(BNO08X_CHANNEL_CONTROL, &[
            BNO08X_CMD_SET_FEATURE, BNO08X_REPORT_ROTATION_VECTOR,
            0, 0, 0, q[0], q[1], q[2], q[3], 0, 0, 0, 0, 0, 0, 0, 0,
        ])?;
        thread::sleep(Duration::from_millis(10));
        self.write_packet(BNO08X_CHANNEL_CONTROL, &[
            BNO08X_CMD_SET_FEATURE, BNO08X_REPORT_GYRO_CALIBRATED,
            0, 0, 0, g[0], g[1], g[2], g[3], 0, 0, 0, 0, 0, 0, 0, 0,
        ])
    }

    fn shtp_init(&mut self) -> Result<()> {
        self.write_packet(BNO08X_CHANNEL_EXECUTABLE, &[0x01])?;  // soft reset
        thread::sleep(Duration::from_millis(300));
        for _ in 0..20 {  // drain boot advertisements
            let _ = self.read_packet();
            thread::sleep(Duration::from_millis(5));
        }
        self.configure_reports(10_000)?;  // enable at 100 Hz (less EIO pressure)
        thread::sleep(Duration::from_millis(50));
        Ok(())
    }
}

/// Background poll loop — runs continuously, mirroring the `debug_bno08x` approach.
fn bno08x_poll_thread(
    i2c_device: String,
    addr: u8,
    shared: Arc<Mutex<Bno08xState>>,
    stop: Arc<AtomicBool>,
    init_tx: std::sync::mpsc::SyncSender<Result<()>>,
) {
    let mut dev = match ShtpDevice::new(&i2c_device, addr) {
        Ok(d) => d,
        Err(e) => { let _ = init_tx.send(Err(e)); return; }
    };
    if let Err(e) = dev.shtp_init() {
        let _ = init_tx.send(Err(e)); return;
    }
    let _ = init_tx.send(Ok(()));  // unblock constructor

    let mut needs_reconfigure = false;
    let mut last_data = std::time::Instant::now();
    let mut last_quat_data = std::time::Instant::now();

    while !stop.load(Ordering::Relaxed) {
        // General watchdog: no ch=3 data at all for 1s → full reinit.
        if last_data.elapsed() > Duration::from_millis(1000) {
            let _ = dev.shtp_init();
            needs_reconfigure = false;
            last_data = std::time::Instant::now();
            last_quat_data = std::time::Instant::now();
            continue;
        }

        // Quat-specific watchdog: rotation vector (0x08) causes heavier sensor fusion
        // → more BCM2835 clock-stretch EIO. Gyro packets keep arriving (resetting
        // last_data), so the general watchdog never fires. Track quat separately.
        if last_quat_data.elapsed() > Duration::from_millis(2000) {
            let _ = dev.shtp_init();
            needs_reconfigure = false;
            last_data = std::time::Instant::now();
            last_quat_data = std::time::Instant::now();
            continue;
        }

        match dev.read_packet() {
            None => {
                if needs_reconfigure {
                    needs_reconfigure = false;
                    if dev.configure_reports(10_000).is_err() {
                        needs_reconfigure = true;
                    } else {
                        thread::sleep(Duration::from_millis(100));
                    }
                } else {
                    thread::sleep(Duration::from_millis(1));
                }
            }
            Some((channel, payload)) => {
                if channel == 0 {
                    needs_reconfigure = true;
                    continue;
                }
                if channel != BNO08X_CHANNEL_REPORTS || payload.is_empty() {
                    continue;
                }

                // Parse channel-3 sensor reports
                let mut cursor = 0;
                while cursor + 5 <= payload.len() {
                    match payload[cursor] {
                        0xFB | 0xFA => cursor += 5,
                        _ => break,
                    }
                }

                let mut gyro_update: Option<[f64; 3]> = None;
                let mut quat_update: Option<[f64; 4]> = None;

                while cursor < payload.len() {
                    match payload[cursor] {
                        0xFB | 0xFA => { cursor += 5; }  // timestamp records can appear between reports
                        BNO08X_REPORT_GYRO_CALIBRATED => {
                            if cursor + 10 > payload.len() { break; }
                            let x = i16::from_le_bytes([payload[cursor+4], payload[cursor+5]]) as f64 / 512.0;
                            let y = i16::from_le_bytes([payload[cursor+6], payload[cursor+7]]) as f64 / 512.0;
                            let z = i16::from_le_bytes([payload[cursor+8], payload[cursor+9]]) as f64 / 512.0;
                            gyro_update = Some([x, y, z]);
                            cursor += 10;
                        }
                        BNO08X_REPORT_ROTATION_VECTOR => {
                            if cursor + 14 > payload.len() { break; }
                            let qi = i16::from_le_bytes([payload[cursor+4],  payload[cursor+5]])  as f64 / 16384.0;
                            let qj = i16::from_le_bytes([payload[cursor+6],  payload[cursor+7]])  as f64 / 16384.0;
                            let qk = i16::from_le_bytes([payload[cursor+8],  payload[cursor+9]])  as f64 / 16384.0;
                            let qw = i16::from_le_bytes([payload[cursor+10], payload[cursor+11]]) as f64 / 16384.0;
                            quat_update = Some([qw, qi, qj, qk]);
                            cursor += 14;  // 14 bytes: 4 header + 8 quat + 2 accuracy estimate
                        }
                        _ => break,
                    }
                }

                // Sanity-check parsed values: discard obviously corrupt packets
                if let Some(g) = gyro_update {
                    if g[0].abs() > 50.0 || g[1].abs() > 50.0 || g[2].abs() > 50.0 {
                        gyro_update = None;
                    }
                }
                if let Some(q) = quat_update {
                    let mag = (q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]).sqrt();
                    if (mag - 1.0).abs() > 0.3 {
                        quat_update = None;
                    }
                }

                if gyro_update.is_some() || quat_update.is_some() {
                    let mut state = shared.lock().unwrap();
                    if let Some(g) = gyro_update { state.last_gyro = g; }
                    if let Some(q) = quat_update { state.last_quat = q; }
                    last_data = std::time::Instant::now();
                    if quat_update.is_some() { last_quat_data = std::time::Instant::now(); }
                    // Sleep briefly after a successful read so we don't immediately
                    // re-read while the sensor is preparing its next packet (which
                    // triggers the BCM2835 clock-stretch timeout → EIO → sensor reset).
                    thread::sleep(Duration::from_millis(4));
                }
            }
        }
    }
}

/// IMU controller for BNO08X sensor family (BNO080/085/086).
///
/// A background thread continuously polls the sensor (mirroring the `debug_bno08x`
/// approach), so `read()` simply returns the latest cached values with no I2C overhead.
pub struct Bno08xController {
    shared: Arc<Mutex<Bno08xState>>,
    stop_flag: Arc<AtomicBool>,
    gravity_offset: [f64; 3],
    world_gravity: [f64; 3],
    gyro_history: [[f64; 3]; 2],
    accel_history: [[f64; 3]; 2],
    _thread: thread::JoinHandle<()>,
}

impl Drop for Bno08xController {
    fn drop(&mut self) {
        self.stop_flag.store(true, Ordering::Relaxed);
    }
}

impl Bno08xController {
    pub const DEFAULT_ADDRESS: u8 = 0x4A;
    pub const ALTERNATE_ADDRESS: u8 = 0x4B;

    pub fn new_default() -> Result<Self> {
        Self::new("/dev/i2c-1", Self::ALTERNATE_ADDRESS)
    }

    pub fn new(i2c_device: &str, addr: u8) -> Result<Self> {
        let shared = Arc::new(Mutex::new(Bno08xState {
            last_gyro: [0.0; 3],
            last_quat: [0.0, 0.0, 0.0, 1.0],
        }));
        let stop_flag = Arc::new(AtomicBool::new(false));

        // Synchronous channel with cap=1: constructor blocks until thread signals init done.
        let (init_tx, init_rx) = std::sync::mpsc::sync_channel(1);
        let shared_clone = Arc::clone(&shared);
        let stop_clone = Arc::clone(&stop_flag);
        let i2c_str = i2c_device.to_string();

        let thread_handle = thread::spawn(move || {
            bno08x_poll_thread(i2c_str, addr, shared_clone, stop_clone, init_tx);
        });

        init_rx.recv()
            .context("BNO08X poll thread terminated before finishing init")??;

        Ok(Self {
            shared,
            stop_flag,
            gravity_offset: [0.0; 3],
            world_gravity: [0.0, 0.0, -1.0],
            gyro_history: [[0.0; 3]; 2],
            accel_history: [[0.0; 3]; 2],
            _thread: thread_handle,
        })
    }

    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        self.gravity_offset = offset;
        println!("Set gravity offset: [{:.4}, {:.4}, {:.4}]", offset[0], offset[1], offset[2]);
    }

    pub fn get_gravity_offset(&self) -> [f64; 3] {
        self.gravity_offset
    }

    pub fn set_world_gravity(&mut self, g: [f64; 3]) {
        self.world_gravity = g;
    }

    /// Returns the latest sensor data. Never blocks on I2C — the background thread
    /// handles all communication.
    pub fn read(&mut self) -> Result<ImuData> {
        let state = self.shared.lock().unwrap();
        let last_gyro = state.last_gyro;
        let last_quat = state.last_quat;
        drop(state);

        // Sensor mounted with X+ forward, Y+ left, Z+ up — matches robot frame directly
        let gyro_raw = [last_gyro[0], last_gyro[1], last_gyro[2]];

        let gyro_filtered = [
            median3(self.gyro_history[0][0], self.gyro_history[1][0], gyro_raw[0]),
            median3(self.gyro_history[0][1], self.gyro_history[1][1], gyro_raw[1]),
            median3(self.gyro_history[0][2], self.gyro_history[1][2], gyro_raw[2]),
        ];
        self.gyro_history[0] = self.gyro_history[1];
        self.gyro_history[1] = gyro_raw;

        let grav_sensor = quat_rotate_vec_inverse(last_quat, self.world_gravity);
        let grav_robot = [grav_sensor[0], grav_sensor[1], grav_sensor[2]];
        let accel_final = finalize_gravity(grav_robot, self.gravity_offset);

        let accel_filtered = [
            median3(self.accel_history[0][0], self.accel_history[1][0], accel_final[0]),
            median3(self.accel_history[0][1], self.accel_history[1][1], accel_final[1]),
            median3(self.accel_history[0][2], self.accel_history[1][2], accel_final[2]),
        ];
        self.accel_history[0] = self.accel_history[1];
        self.accel_history[1] = accel_final;

        Ok(ImuData { gyro: gyro_filtered, accel: accel_filtered, quat: last_quat })
    }
}

// --- BMI088 Controller ---

/// IMU controller for BMI088 sensor.
///
/// Uses a Madgwick AHRS filter (via the `bmi088` crate) to fuse accelerometer
/// and gyroscope data into a quaternion, then extracts projected gravity from it.
///
/// Note: The BMI088 has no hardware axis-remapping registers. Configure the
/// physical mounting orientation via `bmi088::AxisRemap` on the inner driver
/// before passing it to this controller, or use `new_with_remap`.
pub struct Bmi088Controller {
    ahrs: Bmi088Ahrs<I2cdev>,
    gravity_offset: [f64; 3],
    world_gravity: [f64; 3],
    last_read: std::time::Instant,
    gyro_history: [[f64; 3]; 2],
    accel_history: [[f64; 3]; 2],
}

impl Bmi088Controller {
    /// Create with default settings on `/dev/i2c-1`.
    /// Axis remap: X and Y inverted, Z unchanged.
    pub fn new_default() -> Result<Self> {
        Self::new_with_remap("/dev/i2c-1", bmi088::AxisRemap {
            axes: [0, 1, 2],
            signs: [-1.0, -1.0, 1.0],
        })
    }

    /// Create on the given I2C device with a custom axis remap.
    ///
    /// Use the remap to compensate for the physical mounting orientation of the
    /// BMI088 on the board (the sensor has no hardware remapping registers).
    pub fn new_with_remap(i2c_device: &str, remap: bmi088::AxisRemap) -> Result<Self> {
        let i2c = I2cdev::new(i2c_device)
            .context(format!("Failed to open I2C device: {}", i2c_device))?;
        let mut imu = Bmi088::new(i2c, Bmi088Config::default())
            .map_err(|e| anyhow::anyhow!("Failed to initialize BMI088: {:?}", e))?;
        imu.set_axis_remap(remap);
        let ahrs = Bmi088Ahrs::new(imu, 0.1);
        Ok(Self {
            ahrs,
            gravity_offset: [0.0; 3],
            world_gravity: [0.0, 0.0, -1.0],
            last_read: std::time::Instant::now(),
            gyro_history: [[0.0; 3]; 2],
            accel_history: [[0.0; 3]; 2],
        })
    }

    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        self.gravity_offset = offset;
        println!("Set gravity offset: [{:.4}, {:.4}, {:.4}]", offset[0], offset[1], offset[2]);
    }

    pub fn get_gravity_offset(&self) -> [f64; 3] {
        self.gravity_offset
    }

    pub fn set_world_gravity(&mut self, g: [f64; 3]) {
        self.world_gravity = g;
    }

    pub fn read(&mut self) -> Result<ImuData> {
        let now = std::time::Instant::now();
        let dt = now.duration_since(self.last_read).as_secs_f32().clamp(0.001, 0.5);
        self.last_read = now;

        let (gyro_raw_f32, quat_f32) = self.ahrs.update(dt)
            .map_err(|e| anyhow::anyhow!("Failed to read BMI088: {:?}", e))?;

        let gyro_raw = [
            gyro_raw_f32[0] as f64,
            gyro_raw_f32[1] as f64,
            gyro_raw_f32[2] as f64,
        ];
        let quat = [
            quat_f32[0] as f64,
            quat_f32[1] as f64,
            quat_f32[2] as f64,
            quat_f32[3] as f64,
        ];

        // Madgwick quaternion is body→world; apply inverse to project world gravity into body frame
        let accel_raw = quat_rotate_vec_inverse(quat, self.world_gravity);
        let accel_final = finalize_gravity(accel_raw, self.gravity_offset);

        let gyro_filtered = [
            median3(self.gyro_history[0][0], self.gyro_history[1][0], gyro_raw[0]),
            median3(self.gyro_history[0][1], self.gyro_history[1][1], gyro_raw[1]),
            median3(self.gyro_history[0][2], self.gyro_history[1][2], gyro_raw[2]),
        ];
        self.gyro_history[0] = self.gyro_history[1];
        self.gyro_history[1] = gyro_raw;

        let accel_filtered = [
            median3(self.accel_history[0][0], self.accel_history[1][0], accel_final[0]),
            median3(self.accel_history[0][1], self.accel_history[1][1], accel_final[1]),
            median3(self.accel_history[0][2], self.accel_history[1][2], accel_final[2]),
        ];
        self.accel_history[0] = self.accel_history[1];
        self.accel_history[1] = accel_final;

        Ok(ImuData { gyro: gyro_filtered, accel: accel_filtered, quat })
    }
}

// --- Unified IMU controller ---

/// Wraps a BNO055, BNO08X, or BMI088 controller behind a common interface
pub enum AnyImuController {
    Bno055(ImuController),
    Bno08x(Bno08xController),
    Bmi088(Bmi088Controller),
}

impl AnyImuController {
    pub fn read(&mut self) -> Result<ImuData> {
        match self {
            AnyImuController::Bno055(c) => c.read(),
            AnyImuController::Bno08x(c) => c.read(),
            AnyImuController::Bmi088(c) => c.read(),
        }
    }

    pub fn set_gravity_offset(&mut self, offset: [f64; 3]) {
        match self {
            AnyImuController::Bno055(c) => c.set_gravity_offset(offset),
            AnyImuController::Bno08x(c) => c.set_gravity_offset(offset),
            AnyImuController::Bmi088(c) => c.set_gravity_offset(offset),
        }
    }

    pub fn set_world_gravity(&mut self, g: [f64; 3]) {
        match self {
            AnyImuController::Bno055(c) => c.set_world_gravity(g),
            AnyImuController::Bno08x(c) => c.set_world_gravity(g),
            AnyImuController::Bmi088(c) => c.set_world_gravity(g),
        }
    }

    pub fn get_gravity_offset(&self) -> [f64; 3] {
        match self {
            AnyImuController::Bno055(c) => c.get_gravity_offset(),
            AnyImuController::Bno08x(c) => c.get_gravity_offset(),
            AnyImuController::Bmi088(c) => c.get_gravity_offset(),
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
