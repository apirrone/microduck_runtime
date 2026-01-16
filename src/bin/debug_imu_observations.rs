use anyhow::Result;
use clap::Parser;
use std::io::Write;
use std::thread;
use std::time::Duration;
use microduck_runtime::imu::ImuController;

/// IMU Policy Observations Monitor
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Pitch offset in radians to apply to projected gravity (workaround for uncalibrated accelerometer)
    #[arg(long, default_value_t = 0.0)]
    pitch_offset: f64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║         IMU Policy Observations Monitor                       ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("This tool displays the EXACT observations sent to the policy:");
    println!("  - Angular velocity (gyro): rad/s in robot frame");
    println!("  - Projected gravity: normalized unit vector in robot frame");
    println!();
    println!("Robot frame: X=forward, Y=left, Z=up");
    if args.pitch_offset != 0.0 {
        println!("⚠  Using pitch offset: {:.4} rad ({:.2}°)", args.pitch_offset, args.pitch_offset.to_degrees());
    }
    println!();

    // Initialize IMU controller (uses bno055 crate with hardware remapping)
    println!("Initializing BNO055...");
    let mut imu = ImuController::new_default()?;
    println!("✓ BNO055 initialized with hardware axis remapping");
    println!();

    // Wait for calibration
    println!("Waiting for calibration...");
    let mut final_calib = (0, 0, 0, 0);
    for i in 0..20 {
        let (sys, gyro, accel, mag) = imu.get_calibration_status()?;
        final_calib = (sys, gyro, accel, mag);
        print!("\rCalibration: Sys={} Gyro={} Accel={} Mag={}", sys, gyro, accel, mag);
        std::io::stdout().flush()?;

        if sys >= 2 && gyro >= 2 && accel >= 2 {
            println!("\n✓ Calibration sufficient (Sys={} Gyro={} Accel={} Mag={})", sys, gyro, accel, mag);
            break;
        }

        if i == 19 {
            println!("\n⚠ Warning: Calibration incomplete (Sys={} Gyro={} Accel={} Mag={}), but continuing...", sys, gyro, accel, mag);
        }

        thread::sleep(Duration::from_millis(500));
    }
    println!();

    println!("Reading policy observations (Ctrl+C to stop):");
    println!();
    println!("┌─────────────────────────────────────────────────────────────────────────────────────┐");
    println!("│ Angular Velocity (rad/s)          Projected Gravity (normalized)        Magnitude  │");
    println!("│ Gyro_X   Gyro_Y   Gyro_Z          ProjGrav_X  ProjGrav_Y  ProjGrav_Z    (verify)  │");
    println!("├─────────────────────────────────────────────────────────────────────────────────────┤");

    loop {
        // Read IMU data - THIS IS WHAT THE POLICY RECEIVES
        let mut imu_data = imu.read()?;

        // Apply pitch offset if specified (rotation around Y axis)
        if args.pitch_offset != 0.0 {
            let cos_pitch = args.pitch_offset.cos();
            let sin_pitch = args.pitch_offset.sin();
            let x = imu_data.accel[0];
            let z = imu_data.accel[2];

            // Rotate projected gravity around Y axis
            imu_data.accel[0] = x * cos_pitch + z * sin_pitch;
            imu_data.accel[2] = -x * sin_pitch + z * cos_pitch;

            // Renormalize
            let mag = (imu_data.accel[0].powi(2) + imu_data.accel[1].powi(2) + imu_data.accel[2].powi(2)).sqrt();
            if mag > 0.01 {
                imu_data.accel[0] /= mag;
                imu_data.accel[1] /= mag;
                imu_data.accel[2] /= mag;
            }
        }

        // Verify projected gravity is normalized
        let mag = (imu_data.accel[0].powi(2) +
                   imu_data.accel[1].powi(2) +
                   imu_data.accel[2].powi(2)).sqrt();

        // Print observations
        println!("│ {:7.4}  {:7.4}  {:7.4}        {:8.4}    {:8.4}    {:8.4}      {:.4}   │",
                 imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
                 imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                 mag);

        // Warning if magnitude is not close to 1.0
        if (mag - 1.0).abs() > 0.1 {
            println!("│ ⚠ WARNING: Projected gravity magnitude is {:.3}, expected ~1.0                    │", mag);
        }

        thread::sleep(Duration::from_millis(100)); // 10 Hz
    }
}
