use anyhow::{Context, Result};
use microduck_runtime::motor::{MotorController, MOTOR_IDS, NUM_MOTORS};

const MOTOR_NAMES: [&str; NUM_MOTORS] = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
];

fn main() -> Result<()> {
    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    let voltages = motor_controller.read_voltages()
        .context("Failed to read voltages")?;

    println!("{:<20} {:>4}  {:>7}", "Motor", "ID", "Voltage");
    println!("{}", "-".repeat(36));
    for i in 0..NUM_MOTORS {
        println!("{:<20} {:>4}  {:>6.2}V", MOTOR_NAMES[i], MOTOR_IDS[i], voltages[i]);
    }

    Ok(())
}
