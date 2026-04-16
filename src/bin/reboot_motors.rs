use anyhow::{Context, Result};
use microduck_runtime::motor::MotorController;
use std::time::Duration;

fn main() -> Result<()> {
    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    motor_controller.reboot_all()
        .context("Failed to reboot motors")?;

    println!("All motors rebooted. Waiting for them to come back online...");
    std::thread::sleep(Duration::from_millis(500));
    println!("Done.");

    Ok(())
}
