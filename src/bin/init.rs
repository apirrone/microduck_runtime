use anyhow::{Context, Result};
use clap::Parser;
use microduck_runtime::motor::{MotorController, MOTOR_IDS, NEW_LEGS_HYBRID_DEFAULT_POSITION, NUM_MOTORS};
use std::time::Duration;

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
    "mouth_motor",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
];

#[derive(Parser, Debug)]
struct Args {
    /// Continuously print current DOF positions instead of initializing
    #[arg(long)]
    watch: bool,

    /// Use the new_legs_hybrid robot variant default pose (flat legs).
    #[arg(long)]
    new_legs_hybrid: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let mut motor_controller = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;

    if args.watch {
        loop {
            match motor_controller.read_state() {
                Ok(state) => {
                    // Clear screen and move cursor to top
                    print!("\x1B[2J\x1B[H");
                    println!("{:<20} {:>10}  (id)", "name", "pos (rad)");
                    println!("{}", "-".repeat(38));
                    for i in 0..NUM_MOTORS {
                        println!(
                            "{:<20} {:>10.4}  ({})",
                            MOTOR_NAMES[i],
                            state.positions[i],
                            MOTOR_IDS[i]
                        );
                    }
                }
                Err(e) => eprintln!("Read error: {e}"),
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    } else {
        motor_controller.set_torque_enable(true)
            .context("Failed to enable motor torque")?;

        if args.new_legs_hybrid {
            println!("Moving to new_legs_hybrid default pose over 1 second...");
            motor_controller
                .interpolate_to_position(&NEW_LEGS_HYBRID_DEFAULT_POSITION, Duration::from_secs(1))
                .context("Failed to interpolate to new_legs_hybrid default position")?;
        } else {
            println!("Moving to default pose over 1 second...");
            motor_controller.interpolate_to_default(Duration::from_secs(1))
                .context("Failed to interpolate to default position")?;
        }

        println!("Motors initialized to default pose.");
    }

    Ok(())
}
