use anyhow::Result;
use std::thread;
use std::time::Duration;

// Import modules directly from main crate
use microduck_runtime::imu::ImuController;

/// Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in radians
/// Using ZYX intrinsic rotation order (yaw-pitch-roll)
fn quat_to_euler(quat: [f64; 4]) -> [f64; 3] {
    let [w, x, y, z] = quat;

    // Roll (rotation around X axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (rotation around Y axis)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f64::consts::FRAC_PI_2.copysign(sinp) // use 90 degrees if out of range
    } else {
        sinp.asin()
    };

    // Yaw (rotation around Z axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
}

/// Quaternion multiplication: result = q1 * q2
/// Both quaternions in [w, x, y, z] format
fn quat_multiply(q1: [f64; 4], q2: [f64; 4]) -> [f64; 4] {
    let [w1, x1, y1, z1] = q1;
    let [w2, x2, y2, z2] = q2;

    [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]
}

/// Rotate a vector by a quaternion (same as in imu.rs)
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

/// Read quaternion directly from IMU (duplicate of read() but returns more data)
/// Returns: (accel_raw, gyro, quat, projected_gravity)
fn read_imu_detailed(imu: &mut ImuController) -> Result<([f64; 3], [f64; 3], [f64; 4], [f64; 3])> {
    use std::fs::OpenOptions;
    use std::io::{Read, Write};
    use std::os::unix::io::AsRawFd;

    // This is a bit hacky, but we need to access the raw quaternion
    // Let's use the public read() method and reconstruct what we need
    // For now, we'll read the quaternion registers directly

    // Open I2C device
    let i2c_device = "/dev/i2c-1";
    let address = 0x28u8;
    let mut i2c = OpenOptions::new()
        .read(true)
        .write(true)
        .open(i2c_device)?;

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

    // Read accelerometer data (raw sensor measurement, includes gravity + linear acceleration)
    const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
    let mut accel_buffer = [0u8; 6];
    i2c.write(&[BNO055_ACC_DATA_X_LSB])?;
    thread::sleep(Duration::from_micros(100));
    i2c.read(&mut accel_buffer)?;

    // BNO055 accelerometer scale: 1 LSB = 1 m/s² (in m/s² mode, which is default in NDOF)
    let accel_raw = [
        i16::from_le_bytes([accel_buffer[0], accel_buffer[1]]) as f64 / 100.0,  // 1 LSB = 0.01 m/s²
        i16::from_le_bytes([accel_buffer[2], accel_buffer[3]]) as f64 / 100.0,
        i16::from_le_bytes([accel_buffer[4], accel_buffer[5]]) as f64 / 100.0,
    ];

    // Read gyroscope data
    const BNO055_GYR_DATA_X_LSB: u8 = 0x14;
    let mut gyro_buffer = [0u8; 6];
    i2c.write(&[BNO055_GYR_DATA_X_LSB])?;
    thread::sleep(Duration::from_micros(100));
    i2c.read(&mut gyro_buffer)?;

    let scale = 1.0 / 16.0 * std::f64::consts::PI / 180.0;
    let gyro = [
        i16::from_le_bytes([gyro_buffer[0], gyro_buffer[1]]) as f64 * scale,
        i16::from_le_bytes([gyro_buffer[2], gyro_buffer[3]]) as f64 * scale,
        i16::from_le_bytes([gyro_buffer[4], gyro_buffer[5]]) as f64 * scale,
    ];

    // Read quaternion data
    const BNO055_QUA_DATA_W_LSB: u8 = 0x20;
    let mut quat_buffer = [0u8; 8];
    i2c.write(&[BNO055_QUA_DATA_W_LSB])?;
    thread::sleep(Duration::from_micros(100));
    i2c.read(&mut quat_buffer)?;

    let scale = 1.0 / 16384.0;
    let quat = [
        i16::from_le_bytes([quat_buffer[0], quat_buffer[1]]) as f64 * scale, // w
        i16::from_le_bytes([quat_buffer[2], quat_buffer[3]]) as f64 * scale, // x
        i16::from_le_bytes([quat_buffer[4], quat_buffer[5]]) as f64 * scale, // y
        i16::from_le_bytes([quat_buffer[6], quat_buffer[7]]) as f64 * scale, // z
    ];

    // Compute projected gravity using direct quaternion (not conjugate)
    let world_gravity = [0.0, 0.0, -9.81];
    let projected_gravity = quat_rotate_vec(quat, world_gravity);

    Ok((accel_raw, gyro, quat, projected_gravity))
}

fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║          BNO055 IMU Detailed Debug Tool                       ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("This tool helps verify IMU data orientation and units.");
    println!();
    println!("Expected robot frame convention:");
    println!("  X = forward, Y = left, Z = up");
    println!("  Roll = rotation around X (forward) axis");
    println!("  Pitch = rotation around Y (left) axis");
    println!("  Yaw = rotation around Z (up) axis");
    println!();
    println!("BNO055 datasheet frame (Android convention):");
    println!("  X = right, Y = forward, Z = up (when looking at chip)");
    println!();

    println!("Initializing BNO055 on /dev/i2c-1 at address 0x28...");
    let mut imu = ImuController::new_default()?;
    println!("✓ BNO055 initialized successfully");
    println!();

    println!("Waiting for calibration...");
    println!("Calibration status: System | Gyro | Accel | Mag (0-3, 3=fully calibrated)");
    println!();

    // Wait for reasonable calibration
    for _ in 0..20 {
        let (sys, gyro, accel, mag) = imu.get_calibration_status()?;
        println!("Calibration: {:2} | {:2} | {:2} | {:2}", sys, gyro, accel, mag);

        if sys >= 2 && gyro >= 2 {
            println!("✓ Calibration sufficient for testing");
            println!();
            break;
        }

        thread::sleep(Duration::from_millis(500));
    }

    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║ Testing Instructions                                           ║");
    println!("╠════════════════════════════════════════════════════════════════╣");
    println!("║ 1. Place robot on flat surface (this is the zero orientation) ║");
    println!("║ 2. Tilt forward/back - pitch should change (Y axis rotation)  ║");
    println!("║ 3. Tilt left/right - roll should change (X axis rotation)     ║");
    println!("║ 4. Rotate around vertical - yaw should change (Z axis)        ║");
    println!("║ 5. Check projected gravity points down when upright (~-9.81)  ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();

    println!("Reading detailed IMU data (Ctrl+C to stop):");
    println!();
    println!("Format:");
    println!("  Gyro: angular velocity in rad/s (should be ~0 when still)");
    println!("  Quat: orientation quaternion [w, x, y, z] (normalized, w²+x²+y²+z²≈1)");
    println!("  Euler: [roll, pitch, yaw] in degrees");
    println!("  ProjGrav: gravity in body frame (should be [0, 0, -9.81] when upright)");
    println!();

    loop {
        // Read raw sensor data for display
        let accel_raw = imu.read_raw_accelerometer()?;
        let (_, gyro_raw, quat, _) = read_imu_detailed(&mut imu)?;

        // Read the ACTUAL data that the policy sees (transformed in imu.rs)
        let imu_data = imu.read()?;

        // Convert quaternion to Euler angles (in sensor frame)
        let euler = quat_to_euler(quat);
        let euler_deg = [
            euler[0] * 180.0 / std::f64::consts::PI,
            euler[1] * 180.0 / std::f64::consts::PI,
            euler[2] * 180.0 / std::f64::consts::PI,
        ];

        // Transform quaternion to robot frame
        // Rotation of -90° around Z axis: [sensor_Y, -sensor_X, sensor_Z]
        let q_mount = [
            std::f64::consts::FRAC_1_SQRT_2,  // cos(-90°/2) = √2/2
            0.0,
            0.0,
            -std::f64::consts::FRAC_1_SQRT_2, // sin(-90°/2) = -√2/2
        ];
        let quat_robot = quat_multiply(quat, q_mount);

        // Convert transformed quaternion to Euler angles (in robot frame)
        let euler_robot = quat_to_euler(quat_robot);
        let euler_robot_deg = [
            euler_robot[0] * 180.0 / std::f64::consts::PI,
            euler_robot[1] * 180.0 / std::f64::consts::PI,
            euler_robot[2] * 180.0 / std::f64::consts::PI,
        ];

        // Check quaternion magnitude
        let quat_mag = (quat[0].powi(2) + quat[1].powi(2) + quat[2].powi(2) + quat[3].powi(2)).sqrt();

        // Check raw accelerometer magnitude
        let accel_raw_mag = (accel_raw[0].powi(2) + accel_raw[1].powi(2) + accel_raw[2].powi(2)).sqrt();

        // Clear screen (optional, comment out if too jumpy)
        // print!("\x1B[2J\x1B[1;1H");

        println!("┌─────────────────────────────────────────────────────────────┐");
        println!("│ SENSOR FRAME (BNO055 raw readings):                        │");
        println!("│ Accel_raw (m/s²):  X={:7.3}  Y={:7.3}  Z={:7.3} mag={:5.2} │",
                 accel_raw[0], accel_raw[1], accel_raw[2], accel_raw_mag);
        println!("│ Gyro (rad/s):      X={:7.3}  Y={:7.3}  Z={:7.3}      │",
                 gyro_raw[0], gyro_raw[1], gyro_raw[2]);
        println!("│ Quaternion [WXYZ]: {:6.3} {:6.3} {:6.3} {:6.3} (mag={:.3}) │",
                 quat[0], quat[1], quat[2], quat[3], quat_mag);
        println!("│ Euler (deg):       Roll={:7.2}° Pitch={:7.2}° Yaw={:7.2}° │",
                 euler_deg[0], euler_deg[1], euler_deg[2]);

        println!("├─────────────────────────────────────────────────────────────┤");
        println!("│ ROBOT FRAME (X=fwd, Y=left, Z=up) - POLICY INPUT:          │");
        println!("│ Gyro (rad/s):       X={:7.3}  Y={:7.3}  Z={:7.3}      │",
                 imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
        println!("│ ProjGrav (norm):    X={:7.3}  Y={:7.3}  Z={:7.3}      │",
                 imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        println!("│ Euler_robot (deg):  Roll={:7.2}° Pitch={:7.2}° Yaw={:7.2}° │",
                 euler_robot_deg[0], euler_robot_deg[1], euler_robot_deg[2]);

        println!("└─────────────────────────────────────────────────────────────┘");
        println!();

        // Provide real-time hints based on data
        if imu_data.gyro[0].abs() > 0.1 || imu_data.gyro[1].abs() > 0.1 || imu_data.gyro[2].abs() > 0.1 {
            println!("→ Motion detected! Gyro: [{:.2}, {:.2}, {:.2}] rad/s",
                     imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
        }

        if (quat_mag - 1.0).abs() > 0.1 {
            println!("⚠ Warning: Quaternion magnitude is {:.3}, expected ~1.0", quat_mag);
        }

        // Verify projected gravity is normalized (policy expects unit vector)
        let proj_grav_mag = (imu_data.accel[0].powi(2) + imu_data.accel[1].powi(2) + imu_data.accel[2].powi(2)).sqrt();
        if (proj_grav_mag - 1.0).abs() > 0.1 {
            println!("⚠ Warning: Projected gravity magnitude is {:.3}, expected ~1.0 (unit vector)", proj_grav_mag);
        }

        if (accel_raw_mag - 9.81).abs() > 0.5 {
            println!("→ Raw accel magnitude: {:.2} m/s² (not ~9.81)", accel_raw_mag);
            println!("  This suggests LINEAR ACCELERATION during motion");
            println!("  This is NORMAL when manually moving the robot!");
        }

        thread::sleep(Duration::from_millis(200)); // 5 Hz for readability
    }
}
