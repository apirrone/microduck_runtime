use anyhow::Result;
use clap::Parser;
use microduck_runtime::imu::{ImuController, Bno08xController, Bmi088Controller, AnyImuController};
use std::thread;
use std::time::Duration;

#[derive(Parser, Debug)]
#[command(about = "Test IMU sensor readings (gyro + projected gravity)")]
struct Args {
    /// Use BNO08X IMU (BNO080/085/086) instead of the default BNO055
    #[arg(long)]
    bno08x: bool,

    /// Use BMI088 IMU instead of the default BNO055
    #[arg(long)]
    bmi088: bool,

    /// Use projected gravity from quaternion instead of raw accelerometer (BNO055 only)
    #[arg(long)]
    projected_gravity: bool,

    /// Output rate in Hz
    #[arg(long, default_value_t = 10)]
    hz: u32,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("╔════════════════════════════════════════════════════════════════╗");
    if args.bno08x {
        println!("║                   BNO08X IMU Test                            ║");
    } else if args.bmi088 {
        println!("║                   BMI088 IMU Test                            ║");
    } else {
        println!("║                   BNO055 IMU Test                            ║");
    }
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();

    let mut imu = if args.bno08x {
        println!("Initializing BNO08X on /dev/i2c-1 at 0x4B...");
        let ctrl = Bno08xController::new_default()
            .map_err(|e| anyhow::anyhow!("Failed to init BNO08X: {}", e))?;
        println!("✓ BNO08X initialized");
        println!("  Mode: rotation vector (sensor fusion)");
        AnyImuController::Bno08x(ctrl)
    } else if args.bmi088 {
        println!("Initializing BMI088 on /dev/i2c-1...");
        let ctrl = Bmi088Controller::new_default()
            .map_err(|e| anyhow::anyhow!("Failed to init BMI088: {}", e))?;
        println!("✓ BMI088 initialized");
        println!("  Mode: projected gravity from Madgwick filter");
        AnyImuController::Bmi088(ctrl)
    } else {
        println!("Initializing BNO055 on /dev/i2c-1 at 0x29...");
        let ctrl = ImuController::new_default_with_mode(args.projected_gravity)
            .map_err(|e| anyhow::anyhow!("Failed to init BNO055: {}", e))?;
        println!("✓ BNO055 initialized");
        if args.projected_gravity {
            println!("  Mode: projected gravity from quaternion");
        } else {
            println!("  Mode: raw accelerometer");
        }
        AnyImuController::Bno055(ctrl)
    };

    println!();
    println!("Reading at {} Hz. Ctrl+C to stop.", args.hz);
    println!();
    println!("{:>10} {:>10} {:>10} | {:>10} {:>10} {:>10}",
             "Gyro_X", "Gyro_Y", "Gyro_Z", "Grav_X", "Grav_Y", "Grav_Z");
    println!("{:>10} {:>10} {:>10} | {:>10} {:>10} {:>10}",
             "(rad/s)", "(rad/s)", "(rad/s)", "(unit)", "(unit)", "(unit)");
    println!("{}", "=".repeat(75));

    let period = Duration::from_millis(1000 / args.hz as u64);

    loop {
        match imu.read() {
            Ok(data) => {
                println!("{:10.4} {:10.4} {:10.4} | {:10.4} {:10.4} {:10.4}",
                         data.gyro[0], data.gyro[1], data.gyro[2],
                         data.accel[0], data.accel[1], data.accel[2]);
            }
            Err(e) => {
                eprintln!("Read error: {}", e);
            }
        }
        thread::sleep(period);
    }
}
