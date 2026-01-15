use anyhow::Result;
use std::thread;
use std::time::Duration;

// Import modules directly from main crate
use microduck_runtime::imu::ImuController;

fn main() -> Result<()> {
    println!("BNO055 IMU Test Program");
    println!("=======================\n");

    println!("Initializing BNO055 on /dev/i2c-1 at address 0x28...");
    let mut imu = ImuController::new_default()?;
    println!("✓ BNO055 initialized successfully\n");

    println!("Waiting for calibration... (move the sensor around)");
    println!("Calibration status: System | Gyro | Accel | Mag (0-3, 3=fully calibrated)\n");

    // Wait for calibration
    for _ in 0..10 {
        let (sys, gyro, accel, mag) = imu.get_calibration_status()?;
        println!("Calibration: {:2} | {:2} | {:2} | {:2}", sys, gyro, accel, mag);

        if sys == 3 && gyro == 3 && accel == 3 {
            println!("✓ Calibration complete!\n");
            break;
        }

        thread::sleep(Duration::from_millis(500));
    }

    println!("\nReading IMU data (Ctrl+C to stop):\n");
    println!("{:>8} {:>8} {:>8} | {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8}",
             "Gyro X", "Gyro Y", "Gyro Z",
             "AccRaw X", "AccRaw Y", "AccRaw Z",
             "Grav X", "Grav Y", "Grav Z");
    println!("{}", "-".repeat(100));

    loop {
        let raw_accel = imu.read_raw_accelerometer()?;
        let data = imu.read()?;

        println!("{:8.3} {:8.3} {:8.3} | {:8.3} {:8.3} {:8.3} | {:8.3} {:8.3} {:8.3}",
                 data.gyro[0], data.gyro[1], data.gyro[2],
                 raw_accel[0], raw_accel[1], raw_accel[2],
                 data.accel[0], data.accel[1], data.accel[2]);

        thread::sleep(Duration::from_millis(50)); // 20 Hz
    }
}
