use anyhow::{Context, Result};
use microduck_runtime::motor::MotorController;
use std::time::Duration;

fn main() -> Result<()> {
    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    motor_controller.set_torque_enable(false)
        .context("Failed to disable motor torque")?;

    println!("Motor torque disabled.");
    std::thread::sleep(Duration::from_secs(2));

    Ok(())
}
