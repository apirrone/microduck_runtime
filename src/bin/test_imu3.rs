use anyhow::Result;
use std::thread;
use std::time::Duration;
use linux_embedded_hal::{Delay, I2cdev};
use bno055::{Bno055, AxisRemap, BNO055AxisConfig, BNO055AxisSign};

fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║    BNO055 Library Test with Hardware Axis Remapping           ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("Using bno055 crate's built-in axis remapping feature");
    println!();
    println!("Target mapping:");
    println!("  Sensor X (right) → Robot Y (left, negated)");
    println!("  Sensor Y (forward) → Robot X (forward)");
    println!("  Sensor Z (up) → Robot Z (up)");
    println!();

    // Open I2C device
    let i2c = I2cdev::new("/dev/i2c-1")?;
    let mut delay = Delay;

    // Initialize BNO055
    let mut imu = Bno055::new(i2c).with_alternative_address();
    imu.init(&mut delay).map_err(|e| anyhow::anyhow!("{:?}", e))?;

    println!("✓ BNO055 initialized");

    // Configure axis remapping
    // We want: Robot = [Sensor_Y, -Sensor_X, Sensor_Z]
    // Swap X with Y: this makes X read from Y AND Y read from X (bidirectional)
    // Z stays as Z (no swap needed)
    let remap = AxisRemap::builder()
        .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)  // Swap X and Y axes
        .build().map_err(|_| anyhow::anyhow!("Failed to build axis remap"))?;

    imu.set_axis_remap(remap).map_err(|e| anyhow::anyhow!("{:?}", e))?;

    // Flip Y axis sign (right→left)
    imu.set_axis_sign(BNO055AxisSign::Y_NEGATIVE).map_err(|e| anyhow::anyhow!("{:?}", e))?;

    println!("✓ Axis remapping configured: X=Sensor_Y, Y=-Sensor_X, Z=Sensor_Z");
    println!();

    // Set to NDOF mode
    imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay).map_err(|e| anyhow::anyhow!("{:?}", e))?;
    println!("✓ Set to NDOF mode");
    println!();

    println!("Waiting for calibration...");
    for _ in 0..10 {
        let calib = imu.get_calibration_status().map_err(|e| anyhow::anyhow!("{:?}", e))?;
        println!("Calibration: Sys={} Gyro={} Accel={} Mag={}",
                 calib.sys, calib.gyr, calib.acc, calib.mag);
        if calib.sys >= 2 && calib.gyr >= 2 {
            println!("✓ Calibration sufficient");
            break;
        }
        thread::sleep(Duration::from_millis(500));
    }
    println!();

    println!("Reading REMAPPED IMU data (Ctrl+C to stop):");
    println!("Robot frame: X=forward, Y=left, Z=up");
    println!();
    println!("{:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
             "Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z");
    println!("{:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
             "(fwd)", "(left)", "(up)", "(rad/s)", "(rad/s)", "(rad/s)");
    println!("{}", "=".repeat(70));

    loop {
        // Read accelerometer (already remapped by hardware)
        let accel = imu.accel_data().map_err(|e| anyhow::anyhow!("{:?}", e))?;

        // Read gyroscope (already remapped by hardware)
        let gyro = imu.gyro_data().map_err(|e| anyhow::anyhow!("{:?}", e))?;

        // Convert to proper units
        // Accelerometer: mg to m/s² (1g = 9.81 m/s²)
        let acc_x = accel.x as f64 / 1000.0 * 9.81;
        let acc_y = accel.y as f64 / 1000.0 * 9.81;
        let acc_z = accel.z as f64 / 1000.0 * 9.81;

        // Gyroscope: already in rad/s
        let gyro_x = gyro.x as f64;
        let gyro_y = gyro.y as f64;
        let gyro_z = gyro.z as f64;

        println!("{:10.3} {:10.3} {:10.3} {:10.3} {:10.3} {:10.3}",
                 acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

        thread::sleep(Duration::from_millis(100)); // 10 Hz
    }
}
