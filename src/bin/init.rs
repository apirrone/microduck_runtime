use anyhow::{Context, Result};
use microduck_runtime::motor::{MotorController, DEFAULT_POSITION};
use std::time::Duration;

fn main() -> Result<()> {
    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    motor_controller.set_torque_enable(true)
        .context("Failed to enable motor torque")?;

    motor_controller.write_goal_positions(&DEFAULT_POSITION)
        .context("Failed to write initial positions")?;

    println!("Motors initialized to default pose.");
    std::thread::sleep(Duration::from_secs(1));

    Ok(())
}
