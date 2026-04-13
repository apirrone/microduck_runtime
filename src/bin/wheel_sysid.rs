/// Wheel motor system identification for motorized-wheel microduck.
///
/// Runs a series of step-velocity commands on the wheel motors (ankle joints)
/// and records the actual velocity response. Use this to measure:
///   - Step response time constant (how fast velocity tracks command)
///   - Max achievable velocity
///   - Acceleration limits
///
/// IMPORTANT: Run with the robot held in the air or lying on its side so the
/// wheels spin freely. This isolates motor dynamics from robot body dynamics.
///
/// Output: CSV with columns: time_s, cmd_rad_s, left_vel_rad_s, right_vel_rad_s
///
/// Compare with simulation: MuJoCo velocity actuator (kv=1.0, forcerange=±0.75 Nm)
/// should be nearly instantaneous for free-spinning wheel (J ≈ 1e-5 kg·m²).
/// Real Dynamixel bandwidth is much lower due to internal PID and gear dynamics.

use anyhow::{Context, Result};
use clap::Parser;
use microduck_runtime::motor::{
    MotorController, MOTOR_IDS, WHEEL_MOTOR_INDICES,
    OPERATING_MODE_POSITION,
};
use std::fs::File;
use std::io::Write as IoWrite;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(about = "Wheel motor system identification — robot must be off the ground")]
struct Args {
    /// Serial port
    #[arg(short, long, default_value = "/dev/ttyAMA0")]
    port: String,

    /// Baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Output CSV file
    #[arg(short, long, default_value = "wheel_sysid.csv")]
    output: String,

    /// Hold time at each step target before moving to next [seconds]
    #[arg(long, default_value_t = 1.5)]
    hold_time: f64,

    /// Settle time at zero between steps [seconds]
    #[arg(long, default_value_t = 1.0)]
    settle_time: f64,

    /// Sample rate [Hz]
    #[arg(long, default_value_t = 100)]
    rate: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("=== Wheel Motor System Identification ===");
    println!("IMPORTANT: Robot must be off the ground (wheels spinning freely)");
    println!("Output: {}", args.output);
    println!();

    let mut mc = MotorController::new(&args.port, args.baudrate)
        .context("Failed to open motor controller")?;

    // Switch wheel motors to velocity mode
    mc.set_wheel_motors_velocity_mode()
        .context("Failed to set wheel motors to velocity mode")?;
    println!("✓ Wheel motors in velocity mode");

    // Step targets to test [rad/s] — positive and negative
    let step_targets: Vec<f64> = vec![
        2.0, 0.0,
        4.0, 0.0,
        6.0, 0.0,
        8.0, 0.0,
        10.0, 0.0,
        -2.0, 0.0,
        -4.0, 0.0,
        -6.0, 0.0,
        -8.0, 0.0,
        -10.0, 0.0,
    ];

    let dt = Duration::from_micros(1_000_000 / args.rate);
    let hold = Duration::from_secs_f64(args.hold_time);
    let settle = Duration::from_secs_f64(args.settle_time);

    let mut file = File::create(&args.output)
        .context("Failed to create output file")?;
    writeln!(file, "time_s,cmd_rad_s,left_vel_rad_s,right_vel_rad_s,left_id,right_id")?;

    let left_id  = MOTOR_IDS[WHEEL_MOTOR_INDICES[0]];
    let right_id = MOTOR_IDS[WHEEL_MOTOR_INDICES[1]];

    let experiment_start = Instant::now();
    let mut current_cmd = 0.0f64;

    // Send zero velocity first and let settle
    mc.write_wheel_velocities(0.0, 0.0)
        .context("Failed to write zero velocity")?;
    println!("Settling at zero for {:.1}s ...", args.settle_time);
    std::thread::sleep(settle);

    println!("Running {} step tests ...", step_targets.len());
    println!("  target (rad/s) | hold (s)");
    println!("  {:>14} | {:>8}", "target", args.hold_time);

    for &target in &step_targets {
        let label = if target == 0.0 { "settle".to_string() } else { format!("{:+.1}", target) };
        let duration = if target == 0.0 { settle } else { hold };
        println!("  Step → {:<10} for {:.1}s", label, duration.as_secs_f64());

        // Apply step
        mc.write_wheel_velocities(target, target)
            .context("Failed to write wheel velocity")?;
        current_cmd = target;

        // Sample during hold
        let step_start = Instant::now();
        while step_start.elapsed() < duration {
            let loop_start = Instant::now();
            let t = experiment_start.elapsed().as_secs_f64();

            match mc.read_wheel_velocities() {
                Ok((left_vel, right_vel)) => {
                    writeln!(file, "{:.6},{:.3},{:.6},{:.6},{},{}",
                             t, current_cmd, left_vel, right_vel, left_id, right_id)?;
                }
                Err(e) => eprintln!("Read error: {}", e),
            }

            let elapsed = loop_start.elapsed();
            if elapsed < dt {
                std::thread::sleep(dt - elapsed);
            }
        }
    }

    // Stop wheels
    mc.write_wheel_velocities(0.0, 0.0)
        .context("Failed to stop wheels")?;
    println!("Wheels stopped.");

    // Flush CSV
    file.flush()?;
    println!("✓ Data saved to {}", args.output);

    // Restore position mode
    mc.set_wheel_motors_position_mode()
        .context("Failed to restore position mode")?;
    println!("✓ Wheel motors restored to position control (Operating Mode {})",
             OPERATING_MODE_POSITION);

    // Quick analysis: report measured max velocity and rough time constant per step
    println!();
    println!("=== Quick Analysis ===");
    println!("Run the following to analyse and plot:");
    println!("  python3 scripts/analyse_wheel_sysid.py {}", args.output);

    Ok(())
}
