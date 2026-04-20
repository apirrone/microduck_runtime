use anyhow::{Context, Result};
use clap::Parser;
use microduck_runtime::imu::{AnyImuController, Bmi088Controller, Bno08xController, ImuController};
use microduck_runtime::motor::{MotorController, DEFAULT_POSITION};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(about = "La poule: chicken-head trick — stabilize head orientation (and optionally position) from body IMU")]
struct Args {
    /// Use BNO08X IMU (BNO080/085/086) instead of the default BMI088
    #[arg(long)]
    bno08x: bool,

    /// Use BNO055 IMU instead of the default BMI088
    #[arg(long)]
    bno055: bool,

    /// Control loop rate in Hz
    #[arg(long, default_value_t = 100)]
    hz: u32,

    // ─── Rotation compensation ────────────────────────────────────────────
    /// Per-axis gain applied to body angle → head joint command.
    /// Default -1.0 = counter-rotate. Flip sign or set to 0 to tune.
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    gain_roll: f64,
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    gain_pitch: f64,
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    gain_yaw: f64,

    /// Exponential smoothing factor on commanded joint targets (0..1, higher = more responsive).
    #[arg(long, default_value_t = 0.5)]
    alpha: f64,

    /// IMU settle time before snapping the reference quaternion (seconds).
    #[arg(long, default_value_t = 1.5)]
    settle_secs: f64,

    // ─── Translation compensation ─────────────────────────────────────────
    /// Enable translation compensation via accelerometer double-integration.
    #[arg(long)]
    trans: bool,

    /// Gravity magnitude used to subtract expected gravity from raw accel (m/s²).
    #[arg(long, default_value_t = 9.81)]
    gravity: f64,

    /// Accel bias calibration window at startup (seconds). Robot must be still.
    #[arg(long, default_value_t = 1.5)]
    bias_secs: f64,

    /// Leaky-integrator decay time constants (seconds). Smaller = faster decay
    /// back to zero = less drift, but also less "real" tracking of slow motion.
    #[arg(long, default_value_t = 0.4)]
    vel_tau: f64,
    #[arg(long, default_value_t = 0.4)]
    pos_tau: f64,

    /// Joint response per metre of estimated body translation (rad/m).
    /// Split per axis (body frame) and per joint. Defaults are 0 so you can
    /// enable translation mode without any motion until you tune these in.
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    trans_gain_neck_x: f64,
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    trans_gain_head_x: f64,
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    trans_gain_neck_z: f64,
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    trans_gain_head_z: f64,

    /// Clamp on the translation-compensation contribution (rad, each axis).
    #[arg(long, default_value_t = 0.4)]
    trans_clamp: f64,
}

// ── Quaternion helpers (scalar-first [w, x, y, z]) ──────────────────────

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

/// Rotate a body-frame vector to world frame: v_world = q · v_body · q⁻¹.
fn quat_rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let tx = 2.0 * (y * v[2] - z * v[1]);
    let ty = 2.0 * (z * v[0] - x * v[2]);
    let tz = 2.0 * (x * v[1] - y * v[0]);
    [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]
}

/// Rotate a world-frame vector into body frame: v_body = q⁻¹ · v_world · q.
fn quat_rotate_inverse(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    quat_rotate(quat_conjugate(q), v)
}

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

    // ── Motors: enable torque and interpolate to default pose ──────────
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

    // ── IMU init ─────────────────────────────────────────────────────────
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

    // Settle filter, capture reference orientation.
    println!("Settling IMU for {:.1}s...", args.settle_secs);
    let settle_start = Instant::now();
    while settle_start.elapsed() < Duration::from_secs_f64(args.settle_secs) {
        let _ = imu.read();
        std::thread::sleep(Duration::from_millis(10));
    }
    let q_ref = imu.read().context("Initial IMU read failed")?.quat;
    let q_ref_inv = quat_conjugate(q_ref);
    println!(
        "✓ Reference quaternion: [{:.3}, {:.3}, {:.3}, {:.3}]",
        q_ref[0], q_ref[1], q_ref[2], q_ref[3]
    );

    // ── Accelerometer bias calibration (translation mode only) ─────────
    //
    // At rest, raw accel measures the upward normal force ≈ R_world→body · [0,0,+g].
    // Averaging (raw_accel − expected_gravity_body) over a still window gives
    // the residual sensor bias that the loop subtracts afterwards, so a quiet
    // robot produces a near-zero linear-acceleration signal.
    let mut accel_bias = [0.0; 3];
    let mut trans_enabled = args.trans;
    if trans_enabled {
        match imu.read_raw_accelerometer() {
            Err(e) => {
                eprintln!(
                    "⚠  Translation compensation disabled — IMU does not expose raw accel: {}",
                    e
                );
                trans_enabled = false;
            }
            Ok(_) => {
                println!(
                    "Calibrating accel bias for {:.1}s — keep robot still...",
                    args.bias_secs
                );
                let t0 = Instant::now();
                let mut sum = [0.0; 3];
                let mut n = 0u32;
                while t0.elapsed() < Duration::from_secs_f64(args.bias_secs) {
                    if let (Ok(d), Ok(raw)) = (imu.read(), imu.read_raw_accelerometer()) {
                        // Expected accelerometer reading at rest in body frame:
                        // R_world→body · [0, 0, +g]. Uses the live quat (BMI088 bias
                        // can drift during settle; we want the residual against the
                        // quat Madgwick is actually reporting).
                        let expected =
                            quat_rotate_inverse(d.quat, [0.0, 0.0, args.gravity]);
                        for i in 0..3 {
                            sum[i] += raw[i] - expected[i];
                        }
                        n += 1;
                    }
                    std::thread::sleep(Duration::from_millis(5));
                }
                if n > 0 {
                    for i in 0..3 {
                        accel_bias[i] = sum[i] / n as f64;
                    }
                }
                println!(
                    "✓ Accel bias [{:+.3}, {:+.3}, {:+.3}] m/s² ({} samples)",
                    accel_bias[0], accel_bias[1], accel_bias[2], n
                );
            }
        }
    }

    // ── Control loop ─────────────────────────────────────────────────────
    let hz = args.hz.max(1);
    let period = Duration::from_micros(1_000_000 / hz as u64);
    let mut smoothed_rot = [0.0f64; 3]; // [roll, pitch, yaw] compensation
    let mut vel_world = [0.0f64; 3];
    let mut pos_world = [0.0f64; 3];
    let mut last_tick = Instant::now();
    let mut last_print = Instant::now();

    println!(
        "Running @ {} Hz — rotation {} — translation {}. Ctrl+C to stop.",
        hz,
        if args.gain_roll == 0.0 && args.gain_pitch == 0.0 && args.gain_yaw == 0.0 {
            "off"
        } else {
            "on"
        },
        if trans_enabled { "on" } else { "off" }
    );

    while running.load(Ordering::SeqCst) {
        let step_start = Instant::now();
        let dt = step_start.saturating_duration_since(last_tick).as_secs_f64()
            .clamp(1.0 / 500.0, 0.1);
        last_tick = step_start;

        match imu.read() {
            Ok(d) => {
                // ── Rotation compensation ───────────────────────────────
                let q_rel = quat_mul(q_ref_inv, d.quat);
                let [body_roll, body_pitch, body_yaw] = quat_to_rpy(q_rel);
                let rot_target = [
                    body_roll * args.gain_roll,
                    body_pitch * args.gain_pitch,
                    body_yaw * args.gain_yaw,
                ];
                for i in 0..3 {
                    smoothed_rot[i] += args.alpha * (rot_target[i] - smoothed_rot[i]);
                }
                let head_pitch_rot = smoothed_rot[1].clamp(-1.2, 1.2);
                let head_yaw_rot = smoothed_rot[2].clamp(-1.2, 1.2);
                let head_roll_rot = smoothed_rot[0].clamp(-1.0, 1.0);

                // ── Translation compensation ────────────────────────────
                let mut neck_pitch_trans = 0.0f64;
                let mut head_pitch_trans = 0.0f64;
                if trans_enabled {
                    if let Ok(raw) = imu.read_raw_accelerometer() {
                        // Remove gravity component and static bias.
                        let expected_g =
                            quat_rotate_inverse(d.quat, [0.0, 0.0, args.gravity]);
                        let lin_body = [
                            raw[0] - expected_g[0] - accel_bias[0],
                            raw[1] - expected_g[1] - accel_bias[1],
                            raw[2] - expected_g[2] - accel_bias[2],
                        ];
                        // Integrate in world frame so that later decay doesn't
                        // wash out signal whenever the body rotates.
                        let lin_world = quat_rotate(d.quat, lin_body);

                        // Leaky integrators: state += input·dt − state·dt/τ.
                        // τ→∞ is a pure integrator; small τ snaps back to zero.
                        for i in 0..3 {
                            vel_world[i] += lin_world[i] * dt
                                - vel_world[i] * dt / args.vel_tau.max(1e-3);
                            pos_world[i] += vel_world[i] * dt
                                - pos_world[i] * dt / args.pos_tau.max(1e-3);
                        }

                        // Bring position back into body frame for joint mapping.
                        let pos_body = quat_rotate_inverse(d.quat, pos_world);

                        // Linear mix onto the two pitch-axis joints.
                        let raw_neck = args.trans_gain_neck_x * pos_body[0]
                            + args.trans_gain_neck_z * pos_body[2];
                        let raw_head = args.trans_gain_head_x * pos_body[0]
                            + args.trans_gain_head_z * pos_body[2];
                        neck_pitch_trans =
                            raw_neck.clamp(-args.trans_clamp, args.trans_clamp);
                        head_pitch_trans =
                            raw_head.clamp(-args.trans_clamp, args.trans_clamp);
                    }
                }

                // ── Compose targets ─────────────────────────────────────
                let mut positions = DEFAULT_POSITION;
                positions[5] += neck_pitch_trans; // neck_pitch
                positions[6] += head_pitch_rot + head_pitch_trans; // head_pitch
                positions[7] += head_yaw_rot; // head_yaw
                positions[8] += head_roll_rot; // head_roll

                if let Err(e) = motors.write_goal_positions(&positions) {
                    eprintln!("Motor write error: {}", e);
                }

                if last_print.elapsed() >= Duration::from_millis(250) {
                    last_print = Instant::now();
                    if trans_enabled {
                        println!(
                            "rpy[{:+.2} {:+.2} {:+.2}]  rot[p={:+.2} y={:+.2} r={:+.2}]  pos_w[{:+.3} {:+.3} {:+.3}]m  trans[n={:+.2} h={:+.2}]",
                            body_roll, body_pitch, body_yaw,
                            head_pitch_rot, head_yaw_rot, head_roll_rot,
                            pos_world[0], pos_world[1], pos_world[2],
                            neck_pitch_trans, head_pitch_trans
                        );
                    } else {
                        println!(
                            "rpy[{:+.2} {:+.2} {:+.2}]  rot[p={:+.2} y={:+.2} r={:+.2}]",
                            body_roll, body_pitch, body_yaw,
                            head_pitch_rot, head_yaw_rot, head_roll_rot
                        );
                    }
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
