use anyhow::{Context, Result};
use microduck_runtime::motor::{MotorController, DEFAULT_POSITION};
use std::time::Duration;

fn main() -> Result<()> {
    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    motor_controller.set_torque_enable(true)
        .context("Failed to enable motor torque")?;

    println!("Moving to default pose over 3 seconds...");
    motor_controller.interpolate_to_default(Duration::from_secs(3))
        .context("Failed to interpolate to default position")?;

    println!("Motors initialized to default pose.");

    Ok(())
}
