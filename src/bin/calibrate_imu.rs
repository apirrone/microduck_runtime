use anyhow::{Context, Result};
use bno055::{BNO055AxisConfig, BNO055AxisSign, AxisRemap};
use linux_embedded_hal::{Delay, I2cdev};
use std::fs;
use std::path::PathBuf;
use std::thread;
use std::time::Duration;

const I2C_BUS: &str = "/dev/i2c-1";
const CALIBRATION_FILE: &str = ".config/microduck/imu_calibration.bin";

fn get_calibration_path() -> PathBuf {
    let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
    PathBuf::from(home).join(CALIBRATION_FILE)
}

fn main() -> Result<()> {
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║          BNO055 IMU Calibration Tool                        ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("This tool will guide you through calibrating the IMU.");
    println!("Follow the instructions carefully for best results.");
    println!();

    // Initialize I2C
    let i2c = I2cdev::new(I2C_BUS)
        .context("Failed to open I2C bus. Make sure I2C is enabled.")?;
    let mut delay = Delay {};

    // Initialize BNO055
    println!("Initializing BNO055...");
    let mut imu = bno055::Bno055::new(i2c).with_alternative_address();

    imu.init(&mut delay)
        .map_err(|e| anyhow::anyhow!("Failed to initialize BNO055: {:?}", e))?;

    // Configure axis remapping to match robot coordinate system
    // Robot frame: X=forward, Y=left, Z=up
    // BNO055 mounted: X sensor points backward, Y points right, Z points up
    // Therefore: Robot = [-Sensor_Y, -Sensor_X, Sensor_Z]
    println!("Configuring axis remapping...");
    let remap = AxisRemap::builder()
        .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)
        .build()
        .map_err(|e| anyhow::anyhow!("Failed to build axis remap: {:?}", e))?;

    imu.set_axis_remap(remap)
        .map_err(|e| anyhow::anyhow!("Failed to set axis remap: {:?}", e))?;
    imu.set_axis_sign(BNO055AxisSign::X_NEGATIVE | BNO055AxisSign::Y_NEGATIVE)
        .map_err(|e| anyhow::anyhow!("Failed to set axis sign: {:?}", e))?;

    println!("✓ Axis remapping configured");
    println!();

    // Display calibration instructions
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║                  CALIBRATION INSTRUCTIONS                    ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("The BNO055 requires calibration of 4 sensors:");
    println!();
    println!("1. MAGNETOMETER (Mag)");
    println!("   • Make random movements with the robot");
    println!("   • Move it in figure-8 patterns");
    println!("   • Rotate it around all axes");
    println!();
    println!("2. ACCELEROMETER (Accel) - MOST IMPORTANT!");
    println!("   • Place robot in 6 different orientations");
    println!("   • Hold each position steady for a few seconds:");
    println!("     - Flat on table (standing)");
    println!("     - On left side");
    println!("     - On right side");
    println!("     - On back");
    println!("     - On front");
    println!("     - Upside down (be careful!)");
    println!();
    println!("3. GYROSCOPE (Gyro)");
    println!("   • Keep robot stationary for a few seconds");
    println!();
    println!("4. SYSTEM (Sys)");
    println!("   • Overall fusion calibration");
    println!("   • Automatically calibrated when others are done");
    println!();
    println!("Starting calibration in 3 seconds...");
    println!();
    thread::sleep(Duration::from_secs(3));

    // Calibration loop
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  Sensor    │  Level  │  Status                               ║");
    println!("╠══════════════════════════════════════════════════════════════╣");

    let start_time = std::time::Instant::now();
    let mut last_status = (0, 0, 0, 0);

    loop {
        let calib = imu
            .get_calibration_status()
            .map_err(|e| anyhow::anyhow!("Failed to read calibration status: {:?}", e))?;

        let current_status = (calib.sys, calib.gyr, calib.acc, calib.mag);

        // Only print if status changed
        if current_status != last_status {
            let sys_icon = if calib.sys == 3 { "✓" } else { " " };
            let gyr_icon = if calib.gyr == 3 { "✓" } else { " " };
            let acc_icon = if calib.acc == 3 { "✓" } else { " " };
            let mag_icon = if calib.mag == 3 { "✓" } else { " " };

            print!("\r║ System     │   {}/3   │ {} ", calib.sys, sys_icon);
            print!("                                    ║\n");
            print!("║ Gyroscope  │   {}/3   │ {} ", calib.gyr, gyr_icon);
            print!("                                    ║\n");
            print!("║ Accel      │   {}/3   │ {} ", calib.acc, acc_icon);
            if calib.acc < 3 {
                print!("← Move robot to 6 orientations");
            }
            print!("      ║\n");
            print!("║ Mag        │   {}/3   │ {} ", calib.mag, mag_icon);
            if calib.mag < 3 {
                print!("← Rotate in figure-8 patterns");
            }
            print!("       ║\n");
            print!("╠══════════════════════════════════════════════════════════════╣\n");
            print!("║ Time elapsed: {:3}s                                          ║\n", start_time.elapsed().as_secs());
            print!("╚══════════════════════════════════════════════════════════════╝");

            // Move cursor up to overwrite next iteration
            print!("\x1B[6A");
            std::io::Write::flush(&mut std::io::stdout()).ok();

            last_status = current_status;
        }

        // Check if all sensors are fully calibrated
        if calib.sys == 3 && calib.gyr == 3 && calib.acc == 3 && calib.mag == 3 {
            break;
        }

        thread::sleep(Duration::from_millis(100));
    }

    // Move cursor down past the status display
    println!("\n\n\n\n\n");

    println!();
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║              ✓ CALIBRATION COMPLETE!                        ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("Saving calibration profile...");

    // Read calibration profile from sensor
    let calibration = imu
        .calibration_profile(&mut delay)
        .map_err(|e| anyhow::anyhow!("Failed to read calibration profile: {:?}", e))?;

    // Save to file
    let calib_path = get_calibration_path();

    // Create directory if it doesn't exist
    if let Some(parent) = calib_path.parent() {
        fs::create_dir_all(parent)
            .context("Failed to create calibration directory")?;
    }

    // Write calibration data
    let calib_bytes = calibration.as_bytes();
    fs::write(&calib_path, calib_bytes)
        .context("Failed to write calibration file")?;

    println!("✓ Calibration saved to: {}", calib_path.display());
    println!();
    println!("All programs will now automatically load this calibration.");
    println!("You can recalibrate anytime by running this tool again.");
    println!();
    println!("Calibration complete! Total time: {}s", start_time.elapsed().as_secs());

    Ok(())
}
