use anyhow::{Context, Result};
use clap::Parser;
use microduck_runtime::imu::{AnyImuController, Bmi088Controller, Bno08xController, ImuController};
use microduck_runtime::motor::{MotorController, DEFAULT_POSITION};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(about = "La poule: chicken-head trick — stabilize head orientation (roll/pitch/yaw) from body IMU")]
struct Args {
    /// Use BNO08X IMU (BNO080/085/086) instead of the default BMI088
    #[arg(long)]
    bno08x: bool,

    /// Use BNO055 IMU instead of the default BMI088
    #[arg(long)]
    bno055: bool,

    /// Control loop rate in Hz
    #[arg(long, default_value_t = 50)]
    hz: u32,

    /// Per-axis gain applied to body angle → head command.
    /// Default -1.0 = full inverse (counter-rotate). Flip sign to invert, set to 0 to disable.
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    gain_roll: f64,
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    gain_pitch: f64,
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    gain_yaw: f64,

    /// Exponential smoothing factor on the commanded joint targets (0..1, higher = more responsive).
    #[arg(long, default_value_t = 0.5)]
    alpha: f64,

    /// How long to let the IMU filter settle before capturing the reference quaternion (seconds).
    #[arg(long, default_value_t = 1.5)]
    settle_secs: f64,
}

/// Hamilton quaternion product (scalar-first [w, x, y, z]).
fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Tait-Bryan angles (ZYX convention): roll (X), pitch (Y), yaw (Z) from [w, x, y, z].
fn quat_to_rpy(q: [f64; 4]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        sinp.signum() * std::f64::consts::FRAC_PI_2
    } else {
        sinp.asin()
    };
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw]
}

fn main() -> Result<()> {
    let args = Args::parse();

    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false, Ordering::SeqCst))
            .context("Failed to set Ctrl+C handler")?;
    }

    println!("╔════════════════════════════════════════╗");
    println!("║             LA POULE 🐔               ║");
    println!("╚════════════════════════════════════════╝");

    // --- Motors: enable torque and interpolate to default pose ---
    let mut motors = MotorController::new("/dev/ttyAMA0", 1_000_000)
        .context("Failed to initialize motor controller")?;
    motors
        .set_torque_enable(true)
        .context("Failed to enable motor torque")?;
    println!("Moving to default pose over 1 second...");
    motors
        .interpolate_to_default(Duration::from_secs(1))
        .context("Failed to interpolate to default position")?;
    println!("✓ Motors initialized.");

    // --- IMU init ---
    let mut imu = if args.bno08x {
        println!("Initializing BNO08X on /dev/i2c-1...");
        AnyImuController::Bno08x(
            Bno08xController::new_default().context("Failed to init BNO08X")?,
        )
    } else if args.bno055 {
        println!("Initializing BNO055 on /dev/i2c-1 at 0x29...");
        AnyImuController::Bno055(
            ImuController::new_default().context("Failed to init BNO055")?,
        )
    } else {
        println!("Initializing BMI088 on /dev/i2c-1...");
        AnyImuController::Bmi088(
            Bmi088Controller::new_default().context("Failed to init BMI088")?,
        )
    };

    // Let the fusion filter settle, then snapshot the reference orientation.
    println!("Settling IMU for {:.1}s...", args.settle_secs);
    let settle_start = Instant::now();
    while settle_start.elapsed() < Duration::from_secs_f64(args.settle_secs) {
        let _ = imu.read();
        std::thread::sleep(Duration::from_millis(10));
    }
    let q_ref = imu.read().context("Initial IMU read failed")?.quat;
    let q_ref_inv = quat_conjugate(q_ref);
    println!(
        "✓ Reference quaternion captured: [{:.3}, {:.3}, {:.3}, {:.3}]",
        q_ref[0], q_ref[1], q_ref[2], q_ref[3]
    );

    // --- Control loop ---
    let period = Duration::from_micros(1_000_000 / args.hz.max(1) as u64);
    let mut smoothed = [0.0f64; 3]; // [roll, pitch, yaw] compensation
    let mut last_print = Instant::now();

    println!(
        "Running chicken-head compensation @ {} Hz. Ctrl+C to stop.",
        args.hz
    );
    while running.load(Ordering::SeqCst) {
        let step_start = Instant::now();

        match imu.read() {
            Ok(d) => {
                // Rotation from rest pose to current pose, expressed in the rest/world frame.
                let q_rel = quat_mul(q_ref_inv, d.quat);
                let [body_roll, body_pitch, body_yaw] = quat_to_rpy(q_rel);

                let target = [
                    body_roll * args.gain_roll,
                    body_pitch * args.gain_pitch,
                    body_yaw * args.gain_yaw,
                ];
                for i in 0..3 {
                    smoothed[i] += args.alpha * (target[i] - smoothed[i]);
                }

                // Joint-range clamps (conservative — within mechanical travel).
                let head_pitch_cmd = smoothed[1].clamp(-1.2, 1.2);
                let head_yaw_cmd = smoothed[2].clamp(-1.2, 1.2);
                let head_roll_cmd = smoothed[0].clamp(-1.0, 1.0);

                let mut positions = DEFAULT_POSITION;
                positions[6] += head_pitch_cmd; // head_pitch
                positions[7] += head_yaw_cmd; // head_yaw
                positions[8] += head_roll_cmd; // head_roll

                if let Err(e) = motors.write_goal_positions(&positions) {
                    eprintln!("Motor write error: {}", e);
                }

                if last_print.elapsed() >= Duration::from_millis(250) {
                    last_print = Instant::now();
                    println!(
                        "body rpy [{:+.3} {:+.3} {:+.3}] → head [p={:+.3} y={:+.3} r={:+.3}]",
                        body_roll, body_pitch, body_yaw,
                        head_pitch_cmd, head_yaw_cmd, head_roll_cmd
                    );
                }
            }
            Err(e) => eprintln!("IMU read error: {}", e),
        }

        let elapsed = step_start.elapsed();
        if elapsed < period {
            std::thread::sleep(period - elapsed);
        }
    }

    println!("Stopping — returning head to neutral.");
    let _ = motors.interpolate_to_default(Duration::from_millis(500));
    Ok(())
}
