use anyhow::Result;
use std::thread;
use std::time::Duration;
use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::os::unix::io::AsRawFd;

const I2C_SLAVE: u64 = 0x0703;
const BNO055_ADDR: u8 = 0x28;
const BNO055_ACC_DATA_X_LSB: u8 = 0x08;
const BNO055_GYR_DATA_X_LSB: u8 = 0x14;

fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║          Simple IMU Axis Remapping Test                       ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("BNO055 sensor frame: X=right, Y=forward, Z=up");
    println!("Robot frame:         X=forward, Y=left, Z=up");
    println!();
    println!("Simple remapping:");
    println!("  Robot X (fwd) = Sensor Y");
    println!("  Robot Y (left) = -Sensor X (negated because right→left)");
    println!("  Robot Z (up) = Sensor Z");
    println!();

    // Open I2C device
    let mut i2c = OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/i2c-1")?;

    unsafe {
        libc::ioctl(i2c.as_raw_fd(), I2C_SLAVE, BNO055_ADDR as u64);
    }

    println!("✓ BNO055 initialized");
    println!();

    println!("SENSOR FRAME (raw)                        ROBOT FRAME (remapped)");
    println!("{:>8} {:>8} {:>8} {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
             "Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z",
             "Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z");
    println!("{:>8} {:>8} {:>8} {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
             "(right)", "(fwd)", "(up)", "", "", "",
             "(fwd)", "(left)", "(up)", "", "", "");
    println!("{}", "=".repeat(100));

    loop {
        // Read accelerometer (6 bytes)
        let mut accel_buf = [0u8; 6];
        i2c.write(&[BNO055_ACC_DATA_X_LSB])?;
        i2c.read(&mut accel_buf)?;

        let accel_sensor = [
            i16::from_le_bytes([accel_buf[0], accel_buf[1]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buf[2], accel_buf[3]]) as f64 / 100.0,
            i16::from_le_bytes([accel_buf[4], accel_buf[5]]) as f64 / 100.0,
        ];

        // Read gyroscope (6 bytes)
        let mut gyro_buf = [0u8; 6];
        i2c.write(&[BNO055_GYR_DATA_X_LSB])?;
        i2c.read(&mut gyro_buf)?;

        let scale = 1.0 / 16.0 * std::f64::consts::PI / 180.0;
        let gyro_sensor = [
            i16::from_le_bytes([gyro_buf[0], gyro_buf[1]]) as f64 * scale,
            i16::from_le_bytes([gyro_buf[2], gyro_buf[3]]) as f64 * scale,
            i16::from_le_bytes([gyro_buf[4], gyro_buf[5]]) as f64 * scale,
        ];

        // Apply remapping: Robot = [Sensor_Y, -Sensor_X, Sensor_Z]
        let accel_robot = [
            accel_sensor[1],   // robot X (forward) = sensor Y
            -accel_sensor[0],  // robot Y (left) = -sensor X
            accel_sensor[2],   // robot Z (up) = sensor Z
        ];

        let gyro_robot = [
            gyro_sensor[1],    // robot X (forward) = sensor Y
            -gyro_sensor[0],   // robot Y (left) = -sensor X
            gyro_sensor[2],    // robot Z (up) = sensor Z
        ];

        println!("{:8.3} {:8.3} {:8.3} {:8.3} {:8.3} {:8.3} | {:8.3} {:8.3} {:8.3} {:8.3} {:8.3} {:8.3}",
                 accel_sensor[0], accel_sensor[1], accel_sensor[2],
                 gyro_sensor[0], gyro_sensor[1], gyro_sensor[2],
                 accel_robot[0], accel_robot[1], accel_robot[2],
                 gyro_robot[0], gyro_robot[1], gyro_robot[2]);

        thread::sleep(Duration::from_millis(100)); // 10 Hz
    }
}
