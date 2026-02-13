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
    // IMU mounting: X+ right, Y+ forward, Z+ up
    // Robot frame: X+ forward, Y+ left, Z+ up
    // Mapping: Robot = [+Sensor_Y, -Sensor_X, +Sensor_Z]
    println!("Configuring axis remapping...");
    let remap = AxisRemap::builder()
        .swap_x_with(BNO055AxisConfig::AXIS_AS_Y)
        .build()
        .map_err(|e| anyhow::anyhow!("Failed to build axis remap: {:?}", e))?;

    imu.set_axis_remap(remap)
        .map_err(|e| anyhow::anyhow!("Failed to set axis remap: {:?}", e))?;
    imu.set_axis_sign(BNO055AxisSign::Y_NEGATIVE)
        .map_err(|e| anyhow::anyhow!("Failed to set axis sign: {:?}", e))?;

    println!("✓ Axis remapping configured");

    // Set to IMU mode for calibration (6-axis fusion: accelerometer + gyroscope)
    println!("Setting IMU mode for calibration...");
    imu.set_mode(bno055::BNO055OperationMode::IMU, &mut delay)
        .map_err(|e| anyhow::anyhow!("Failed to set IMU mode: {:?}", e))?;

    println!("✓ IMU mode enabled");

    // Give the sensor a moment to stabilize in IMU mode
    thread::sleep(Duration::from_millis(500));

    // Verify sensors are working
    println!("Verifying sensors are responding...");
    match imu.accel_data() {
        Ok(accel) => println!("  ✓ Accelerometer: [{:.2}, {:.2}, {:.2}] m/s²", accel.x, accel.y, accel.z),
        Err(e) => {
            println!("  ✗ Accelerometer not responding: {:?}", e);
            println!("  Check I2C connection!");
        }
    }
    match imu.gyro_data() {
        Ok(gyro) => println!("  ✓ Gyroscope: [{:.2}, {:.2}, {:.2}] dps/16", gyro.x, gyro.y, gyro.z),
        Err(e) => println!("  ✗ Gyroscope not responding: {:?}", e),
    }
    println!();

    // Display calibration instructions
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║                  CALIBRATION INSTRUCTIONS                    ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("The BNO055 requires calibration of 4 sensors.");
    println!("Follow this order for best results:");
    println!();
    println!("STEP 1: GYROSCOPE (should calibrate within ~5 seconds)");
    println!("   • Place robot on a stable surface");
    println!("   • Keep it COMPLETELY STILL");
    println!("   • Wait for Gyro level to reach 3");
    println!();
    println!("STEP 2: ACCELEROMETER - MOST IMPORTANT!");
    println!("   • After Gyro reaches 3, place robot in 6 orientations:");
    println!("     1. Flat on table (standing) - hold 3 seconds");
    println!("     2. On left side - hold 3 seconds");
    println!("     3. On right side - hold 3 seconds");
    println!("     4. On back - hold 3 seconds");
    println!("     5. On front - hold 3 seconds");
    println!("     6. Upside down - hold 3 seconds");
    println!();
    println!("STEP 3: MAGNETOMETER");
    println!("   • Move away from metal objects and electronics");
    println!("   • Move robot in figure-8 patterns");
    println!("   • Rotate it smoothly around all three axes");
    println!("   • Continue until Mag reaches 3");
    println!();
    println!("STEP 4: SYSTEM (automatic)");
    println!("   • Calibrates automatically when others complete");
    println!();
    println!("IMPORTANT: Start with the robot STILL for gyro calibration!");
    println!("Starting in 5 seconds...");
    println!();
    thread::sleep(Duration::from_secs(5));

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
