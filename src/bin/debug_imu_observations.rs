use anyhow::Result;
use std::io::Write;
use std::thread;
use std::time::Duration;
use microduck_runtime::imu::ImuController;

fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║         IMU Policy Observations Monitor                       ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("This tool displays the EXACT observations sent to the policy:");
    println!("  - Angular velocity (gyro): rad/s in robot frame");
    println!("  - Projected gravity: normalized unit vector in robot frame");
    println!();
    println!("Robot frame: X=forward, Y=left, Z=up");
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
        let imu_data = imu.read()?;

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
