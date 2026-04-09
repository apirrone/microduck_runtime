mod imu;
mod motor;
mod observation;
mod policy;
mod controller;
mod odometry;

use anyhow::{Context, Result};
use clap::Parser;
use imu::{ImuController, Bno08xController, Bmi088Controller, AnyImuController};
use motor::{MotorController, NUM_MOTORS, DEFAULT_POSITION, MOUTH_MOTOR_IDX, MOUTH_MIN_ANGLE, MOUTH_MAX_ANGLE,
            DEFAULT_POSITION_MOTORIZED_WHEEL, WHEEL_MOTOR_INDICES, WHEEL_MAX_VEL, OPERATING_MODE_POSITION};
use observation::Observation;
use policy::Policy;
use controller::Controller;
use serde::Serialize;
use std::fs::File;
use std::io::Write as IoWrite;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Timestamped observation for recording
#[derive(Serialize, Debug, Clone)]
struct TimestampedObservation {
    timestamp: f64,  // Seconds since start
    observation: Vec<f32>,
}

/// Metadata captured once at the start of a recording session
#[derive(Serialize, Debug, Clone)]
struct RecordingMetadata {
    action_scale: f64,
    voltage_v: f64,    // average motor voltage at recording start
    freq_hz: u32,
    unix_time: f64,    // wall-clock seconds since UNIX epoch
}

/// Top-level structure serialised to the .pkl file
#[derive(Serialize, Debug, Clone)]
struct RecordingData {
    metadata: RecordingMetadata,
    observations: Vec<TimestampedObservation>,
}

/// Microduck Robot Runtime
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port for motor communication
    #[arg(short, long, default_value = "/dev/ttyAMA0")]
    port: String,

    /// Motor communication baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Control loop frequency in Hz
    #[arg(short, long, default_value_t = 50)]
    freq: u32,

    /// Use dummy policy (always outputs zeros) for testing
    #[arg(short, long)]
    dummy: bool,

    /// Path to ONNX model file (ignored if --dummy is set)
    #[arg(short, long, default_value = "~/microduck/policies/walking.onnx")]
    model: Option<String>,

    /// Path to standing policy ONNX model file (optional)
    #[arg(short, long, default_value = "~/microduck/policies/standing.onnx")]
    standing: Option<String>,

    /// Position P gain for motors
    #[arg(long, default_value_t = 200)]
    kp: u16,

    /// Position I gain for motors
    #[arg(long, default_value_t = 0)]
    ki: u16,

    /// Position D gain for motors
    #[arg(long, default_value_t = 0)]
    kd: u16,

    /// Pitch offset in radians to apply to projected gravity (workaround for uncalibrated accelerometer)
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pitch_offset: f64,

    /// Gravity offset to subtract from projected gravity [x,y,z]
    /// Compensates for IMU mounting angle or calibration bias
    /// Measure by recording gravity when robot is standing upright
    /// Example: --gravity-offset-x -0.086 (if upright shows g_x=-0.086)
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    gravity_offset_x: f64,

    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    gravity_offset_y: f64,

    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    gravity_offset_z: f64,

    /// Use raw accelerometer instead of projected gravity (default: use projected gravity)
    /// By default, sensor fusion computes clean gravity free from dynamic accelerations.
    /// This flag disables that and uses raw accelerometer readings instead.
    #[arg(long)]
    raw_accelerometer: bool,

    /// Use BNO08X IMU (BNO080/085/086) instead of the default BNO055
    /// Connects via I2C at address 0x4A (default) on /dev/i2c-1
    #[arg(long)]
    bno08x: bool,

    /// Use BMI088 IMU instead of the default BNO055
    /// Connects via I2C on /dev/i2c-1 (accel 0x19, gyro 0x68)
    #[arg(long)]
    bmi088: bool,

    /// Optional CSV log file to save observations and actions
    #[arg(long)]
    log_file: Option<String>,

    /// Linear velocity command in X direction (m/s)
    #[arg(short = 'x', long = "vel_x", default_value_t = 0.0, allow_hyphen_values = true)]
    vel_x: f64,

    /// Linear velocity command in Y direction (m/s)
    #[arg(short = 'y', long = "vel_y", default_value_t = 0.0, allow_hyphen_values = true)]
    vel_y: f64,

    /// Angular velocity command around Z axis (rad/s)
    #[arg(short = 'z', long = "vel_z", default_value_t = 0.0, allow_hyphen_values = true)]
    vel_z: f64,

    /// Action scale multiplier (scales policy outputs before applying to motors)
    #[arg(long, default_value_t = 0.6, allow_hyphen_values = true)]
    action_scale: f64,

    /// Adapt action scale proportionally to battery voltage.
    /// Effective scale = action_scale × (nominal_voltage / measured_voltage).
    /// Compensates for kP varying linearly with supply voltage (BAM: kP ∝ V).
    #[arg(long)]
    voltage_adapt: bool,

    /// Reference (nominal) voltage used during BAM motor identification, in volts.
    /// Only used when --voltage-adapt is set.
    #[arg(long, default_value_t = 7.4)]
    nominal_voltage: f64,

    /// Enable recording mode: save observations to pickle file on Ctrl+C
    #[arg(short, long)]
    record: Option<String>,

    /// Maximum linear velocity for controller input (m/s)
    #[arg(long, default_value_t = 0.4)]
    max_linear_vel: f64,

    /// Maximum angular velocity for controller input (rad/s)
    #[arg(long, default_value_t = 2.0)]
    max_angular_vel: f64,

    /// Controller deadzone (0.0-1.0)
    #[arg(long, default_value_t = 0.1)]
    controller_deadzone: f32,

    /// Default neck pitch offset in radians (added on top of policy output at startup)
    #[arg(long, default_value_t = -0.5, allow_hyphen_values = true)]
    neck_pitch_default: f64,

    /// Default head pitch offset in radians (added on top of policy output at startup)
    #[arg(long, default_value_t = 0.5, allow_hyphen_values = true)]
    head_pitch_default: f64,

    /// Maximum head/neck joint offset in head mode (radians)
    #[arg(long, default_value_t = 2.5)]
    head_max: f64,

    /// Smoothing factor for head commands (0.0=no movement, 1.0=no smoothing)
    #[arg(long, default_value_t = 0.2)]
    head_alpha: f64,

    /// Smoothing factor for velocity commands (0.0=no movement, 1.0=no smoothing)
    #[arg(long, default_value_t = 0.2)]
    cmd_alpha: f64,

    /// Estimate mean and peak current draw across all 14 body motors during the session.
    /// Samples are collected once per second and the results are printed at shutdown.
    #[arg(long)]
    estimate_current: bool,

    /// Roller mode: always use the walking/roller policy, never switch to standing policy
    #[arg(long)]
    roller: bool,

    /// Motorized wheel mode: ankle motors use velocity control (Operating Mode 1).
    /// Observation is 49D (no wheel joint_pos, no mouth). Never switches to standing policy.
    #[arg(long)]
    motorized_wheel: bool,

    /// Battery benchmark mode: robot walks on the spot until battery dies.
    /// Ignores controller input, auto-recovers from falls.
    /// Logs start time, elapsed time and current time every second to the given file.
    #[arg(long)]
    battery_benchmark: Option<String>,

    /// Forward velocity (m/s) used in battery benchmark mode.
    #[arg(long, default_value_t = 0.3)]
    benchmark_vel: f64,

    /// Path to ground pick policy ONNX model file (enables A button one-shot ground pick)
    #[arg(long, default_value = "~/microduck/policies/ground_pick.onnx")]
    ground_pick: Option<String>,

    /// Duration of one ground pick cycle in seconds
    #[arg(long, default_value_t = 4.0)]
    ground_pick_period: f64,

    /// Action scale to use during ground pick (overrides --action-scale for the duration)
    #[arg(long, default_value_t = 1.0)]
    ground_pick_action_scale: f64,

    /// kP multiplier applied to all motors during ground pick (e.g. 0.6 = 40% reduction)
    #[arg(long, default_value_t = 0.6)]
    ground_pick_kp_ratio: f64,

    /// Path to jump policy ONNX model file (enables X button one-shot jump)
    #[arg(long, default_value = "~/microduck/policies/jump.onnx")]
    jump: Option<String>,

    /// Duration of one jump cycle in seconds
    #[arg(long, default_value_t = 1.0)]
    jump_period: f64,

    /// Action scale to use during jump (overrides --action-scale for the duration)
    #[arg(long, default_value_t = 1.0)]
    jump_action_scale: f64,

    /// kP multiplier applied to all motors during jump
    #[arg(long, default_value_t = 1.0)]
    jump_kp_ratio: f64,

    /// kP multiplier applied to all motors when using the standing policy (e.g. 0.6 = 40% reduction)
    #[arg(long, default_value_t = 0.6)]
    standing_kp_ratio: f64,

    /// Enable 15-DOF mode: mouth_motor is part of the policy (use with new policies trained with mouth).
    /// Without this flag, the runtime runs in 14-DOF legacy mode: mouth is excluded from the policy
    /// observation/action and is controlled directly via the right trigger.
    #[arg(long)]
    mouth: bool,

    /// Apply a low-pass filter to head/neck joint commands to reduce oscillation on lightly-loaded joints
    #[arg(long)]
    head_low_pass: bool,

    /// Smoothing factor for head low-pass filter (0.0=frozen, 1.0=no filtering). Default: 0.3
    #[arg(long, default_value_t = 0.3)]
    head_low_pass_alpha: f64,

    /// Apply a low-pass filter to leg joint commands to reduce oscillation
    #[arg(long)]
    legs_low_pass: bool,

    /// Smoothing factor for legs low-pass filter (0.0=frozen, 1.0=no filtering). Default: 0.3
    #[arg(long, default_value_t = 0.3)]
    legs_low_pass_alpha: f64,

    /// Stream robot state (joint angles + IMU quaternion) over TCP for digital twin visualization.
    /// Sends [qw, qx, qy, qz, j0..j14] as 19 × f32 LE (76 bytes/frame) at control loop frequency.
    #[arg(long)]
    stream: bool,

    /// TCP port for digital twin streaming (default: 9870)
    #[arg(long, default_value_t = 9870)]
    stream_port: u16,

    /// Path to robot.urdf for inline odometry (used when --stream is active).
    /// Defaults to ~/microduck/robot.urdf.
    #[arg(long)]
    urdf: Option<String>,
}


/// Main robot runtime
struct Runtime {
    motor_controller: MotorController,
    imu_controller: AnyImuController,
    policy: Policy,
    controller: Controller,
    control_freq: u32,
    pid_gains: (u16, u16, u16), // (kP, kI, kD)
    pitch_offset: f64,
    last_action: [f32; NUM_MOTORS],
    command: [f64; 3],
    action_scale: f64,
    log_file: Option<File>,
    start_time: Option<Instant>,
    step_counter: u64,
    record_file: Option<String>,
    recorded_observations: Vec<TimestampedObservation>,
    record_voltage: f64,   // voltage read at recording start (0.0 if not recording)
    voltage_ema: f64,      // slow EMA of battery voltage, updated every second
    voltage_nominal: f64,  // reference voltage for proportional scaling (BAM nominal)
    voltage_adapt: bool,   // whether to apply voltage-proportional action scale
    policy_enabled: bool,  // Controls whether policy inference runs
    max_linear_vel: f64,
    max_angular_vel: f64,
    controller_deadzone: f32,
    start_button_prev_state: bool,  // Track Start button state for edge detection
    mouth_enabled: bool,  // Whether mouth is part of the policy (--mouth flag)
    mouth_position: f64,  // Legacy mode: absolute mouth position from trigger (not used in mouth mode)
    mouth_offset: f64,    // Mouth mode: additive offset on top of policy mouth output (right trigger)
    head_mode: bool,  // Head control mode (Y button)
    head_offsets: [f64; 4],  // [neck_pitch, head_pitch, head_yaw, head_roll] added on top of policy outputs
    default_positions: [f64; NUM_MOTORS],  // per-run override of DEFAULT_POSITION
    head_max: f64,  // Max head offset in radians
    head_alpha: f64,  // EMA smoothing factor for head commands (0=still, 1=no smoothing)
    cmd_alpha: f64,   // EMA smoothing factor for velocity commands (0=still, 1=no smoothing)
    body_cmd: [f64; 3],  // Body pose commands [z_offset_m, pitch_rad, roll_rad] (normalized before obs)
    y_button_prev_state: bool,  // Track Y button state for edge detection
    body_pose_mode: bool,  // Body pose control mode (B button)
    b_button_prev_state: bool,  // Track B button state for edge detection
    fallen: bool,  // Whether the robot is currently detected as fallen
    fall_detected_since: Option<Instant>,  // When continuous fall detection started (for debounce)
    // Ground pick
    ground_pick_active: bool,
    ground_pick_phase: f64,
    ground_pick_period: f64,
    ground_pick_action_scale: f64,
    ground_pick_kp_ratio: f64,
    prev_action_scale: f64,
    a_button_prev_state: bool,
    // Jump
    jump_active: bool,
    jump_phase: f64,
    jump_period: f64,
    jump_action_scale: f64,
    jump_kp_ratio: f64,
    x_button_prev_state: bool,
    // Current estimation (running stats, updated every control step)
    estimate_current: bool,
    current_sum: f64,
    current_count: u64,
    current_peak: f64,
    // Standing policy
    roller_mode: bool,
    motorized_wheel_mode: bool,
    has_standing_policy: bool,
    is_using_standing: bool,
    standing_kp_ratio: f64,
    standing_prev_action_scale: f64,
    // Emergency stop
    select_held_since: Option<Instant>,
    // Battery benchmark
    battery_benchmark: bool,
    benchmark_vel: f64,
    benchmark_log: Option<File>,
    benchmark_recovering: bool,
    benchmark_start_unix: f64,
    // Head low-pass filter
    head_low_pass: bool,
    head_low_pass_alpha: f64,
    head_low_pass_prev: [f64; 4],  // previous filtered targets for indices 5-8
    // Legs low-pass filter
    legs_low_pass: bool,
    legs_low_pass_alpha: f64,
    legs_low_pass_prev: [f64; 10],  // previous filtered targets for left leg (0-4) and right leg (10-14)
    // Digital twin TCP stream
    stream_listener: Option<std::net::TcpListener>,
    stream_client: Option<std::net::TcpStream>,
    // Inline odometry engine (None if no URDF provided)
    odometry_engine: Option<odometry::Odometry>,
    /// Latest odometry estimate [x, y, z, yaw] in world frame (metres / radians)
    pub odometry: [f64; 4],
}

impl Runtime {
    /// Create a new runtime instance
    fn new(args: &Args) -> Result<Self> {
        println!("Initializing microduck runtime...");

        // Initialize motor controller
        let motor_controller = MotorController::new(&args.port, args.baudrate)
            .context("Failed to initialize motor controller")?;
        println!("✓ Motor controller initialized on {} at {} baud", args.port, args.baudrate);

        // Initialize IMU controller
        let mut imu_controller = if args.bno08x {
            let ctrl = Bno08xController::new_default()
                .context("Failed to initialize IMU controller (BNO08X on /dev/i2c-1 at 0x4B)")?;
            println!("✓ IMU controller initialized (BNO08X) - using projected gravity from rotation vector");
            AnyImuController::Bno08x(ctrl)
        } else if args.bmi088 {
            let ctrl = Bmi088Controller::new_default()
                .context("Failed to initialize IMU controller (BMI088 on /dev/i2c-1)")?;
            println!("✓ IMU controller initialized (BMI088) - using projected gravity from Madgwick filter");
            AnyImuController::Bmi088(ctrl)
        } else {
            let use_projected_gravity = !args.raw_accelerometer;
            let ctrl = ImuController::new_default_with_mode(use_projected_gravity)
                .context("Failed to initialize IMU controller (BNO055 on /dev/i2c-1)")?;
            if use_projected_gravity {
                println!("✓ IMU controller initialized (BNO055) - using projected gravity from quaternion");
            } else {
                println!("✓ IMU controller initialized (BNO055) - using raw accelerometer");
            }
            AnyImuController::Bno055(ctrl)
        };

        // Set gravity offset if specified
        if args.gravity_offset_x != 0.0 || args.gravity_offset_y != 0.0 || args.gravity_offset_z != 0.0 {
            imu_controller.set_gravity_offset([
                args.gravity_offset_x,
                args.gravity_offset_y,
                args.gravity_offset_z,
            ]);
        }

        // Initialize policy based on arguments
        let mut policy = if args.dummy {
            println!("✓ Using dummy policy (always outputs zeros)");
            Policy::new_dummy().context("Failed to create dummy policy")?
        } else if let Some(ref walking_path) = args.model {
            if let Some(ref standing_path) = args.standing {
                println!("✓ Loading walking ONNX model from: {}", walking_path);
                println!("✓ Loading standing ONNX model from: {}", standing_path);
                println!("✓ Policy switching enabled (threshold: 0.05 for cmd magnitude)");
                Policy::new_dual_onnx(walking_path, standing_path)
                    .context("Failed to load dual ONNX models")?
            } else {
                println!("✓ Loading ONNX model from: {}", walking_path);
                Policy::new_onnx(walking_path).context("Failed to load ONNX model")?
            }
        } else {
            println!("! No policy specified, using dummy policy");
            Policy::new_dummy().context("Failed to create dummy policy")?
        };

        // In roller mode, disable standing policy switching
        if args.roller {
            policy.set_standing_disabled(true);
            println!("✓ Roller mode: standing policy switching disabled");
        }

        // In motorized wheel mode, disable standing policy switching
        if args.motorized_wheel {
            policy.set_standing_disabled(true);
            println!("✓ Motorized wheel mode: standing policy switching disabled");
        }

        // Battery benchmark: standing policy is kept active so it can handle fall recovery
        let benchmark_log = if let Some(ref path) = args.battery_benchmark {
            let mut file = File::create(path)
                .context(format!("Failed to create benchmark log file: {}", path))?;
            writeln!(file, "start_time_unix,elapsed_seconds,current_time_unix,avg_voltage")?;
            println!("✓ Battery benchmark mode enabled: log -> {}", path);
            println!("  Controller ignored. Alternates ±0.1 m/s. Auto-recovers from falls via standing policy.");
            Some(file)
        } else {
            None
        };

        // Load ground pick model if specified
        if let Some(ref gp_path) = args.ground_pick {
            println!("✓ Loading ground pick ONNX model from: {}", gp_path);
            policy.add_ground_pick(gp_path)
                .context("Failed to load ground pick ONNX model")?;
            println!("  Ground pick period: {:.2}s (A button to trigger)", args.ground_pick_period);
        }

        // Load jump model if specified
        if let Some(ref jump_path) = args.jump {
            println!("✓ Loading jump ONNX model from: {}", jump_path);
            policy.add_jump(jump_path)
                .context("Failed to load jump ONNX model")?;
            println!("  Jump period: {:.2}s (X button to trigger)", args.jump_period);
        }

        if args.pitch_offset != 0.0 {
            println!("⚠  Using pitch offset: {:.4} rad ({:.2}°)", args.pitch_offset, args.pitch_offset.to_degrees());
        }

        // Velocity command configuration
        if args.vel_x != 0.0 || args.vel_y != 0.0 || args.vel_z != 0.0 {
            println!("✓ Velocity command: x={:.3} m/s, y={:.3} m/s, z={:.3} rad/s",
                     args.vel_x, args.vel_y, args.vel_z);
        }

        // Action scale configuration
        if args.action_scale != 1.0 {
            println!("✓ Action scale: {:.3}", args.action_scale);
        }

        // Recording mode configuration
        if let Some(ref path) = args.record {
            println!("✓ Recording mode enabled: observations will be saved to {}", path);
        }

        // Controller configuration
        println!("Initializing Xbox controller...");
        let mut controller = Controller::new()
            .context("Failed to initialize controller")?;
        if args.battery_benchmark.is_none() {
            controller.wait_for_connection()
                .context("Failed to connect to controller")?;
            if let Some(name) = controller.get_controller_name() {
                println!("✓ Controller connected: {}", name);
            }
        } else {
            println!("⚡ Battery benchmark: skipping controller connection");
        }
        println!("  Max linear velocity: {:.2} m/s", args.max_linear_vel);
        println!("  Max angular velocity: {:.2} rad/s", args.max_angular_vel);
        println!("  Deadzone: {:.2}", args.controller_deadzone);

        // Initialize log file if requested
        let mut log_file = None;
        if let Some(ref path) = args.log_file {
            let mut file = File::create(path)
                .context(format!("Failed to create log file: {}", path))?;

            // Write CSV header: step, time, obs_0..obs_N, action_0..action_13
            let obs_size = 51;
            write!(file, "step,time")?;
            for i in 0..obs_size {
                write!(file, ",obs_{}", i)?;
            }
            for i in 0..14 {
                write!(file, ",action_{}", i)?;
            }
            writeln!(file)?;

            log_file = Some(file);
            println!("✓ CSV logging enabled: {}", path);
        }

        // Policy starts disabled — user presses Start to enable
        let policy_enabled = false;

        Ok(Self {
            motor_controller,
            imu_controller,
            policy,
            controller,
            control_freq: args.freq,
            pid_gains: (args.kp, args.ki, args.kd),
            pitch_offset: args.pitch_offset,
            last_action: [0.0; NUM_MOTORS],
            command: [args.vel_x, args.vel_y, args.vel_z],
            action_scale: args.action_scale,
            log_file,
            start_time: None,
            step_counter: 0,
            record_file: args.record.clone(),
            recorded_observations: Vec::new(),
            record_voltage: 0.0,
            voltage_ema: args.nominal_voltage,
            voltage_nominal: args.nominal_voltage,
            voltage_adapt: args.voltage_adapt,
            policy_enabled,
            max_linear_vel: args.max_linear_vel,
            max_angular_vel: args.max_angular_vel,
            controller_deadzone: args.controller_deadzone,
            start_button_prev_state: false,
            mouth_enabled: args.mouth,
            mouth_position: MOUTH_MIN_ANGLE,
            mouth_offset: 0.0,
            head_mode: false,
            head_offsets: [0.0; 4],
            default_positions: {
                let mut p = DEFAULT_POSITION;
                if args.motorized_wheel {
                    // Fill in the 12 position-controlled joints from the motorized-wheel default.
                    // Skip wheel indices (4, 14) and mouth index (9); map remaining in order.
                    let mut mw_iter = DEFAULT_POSITION_MOTORIZED_WHEEL.iter();
                    for i in 0..NUM_MOTORS {
                        if !WHEEL_MOTOR_INDICES.contains(&i) && i != MOUTH_MOTOR_IDX {
                            if let Some(&v) = mw_iter.next() {
                                p[i] = v;
                            }
                        }
                    }
                }
                p[5] = args.neck_pitch_default;
                p[6] = args.head_pitch_default;
                p
            },
            head_max: args.head_max,
            head_alpha: args.head_alpha,
            cmd_alpha: args.cmd_alpha,
            body_cmd: [0.0; 3],
            y_button_prev_state: false,
            body_pose_mode: false,
            b_button_prev_state: false,
            fallen: false,
            fall_detected_since: None,
            ground_pick_active: false,
            ground_pick_phase: 0.0,
            ground_pick_period: args.ground_pick_period,
            ground_pick_action_scale: args.ground_pick_action_scale,
            ground_pick_kp_ratio: args.ground_pick_kp_ratio,
            prev_action_scale: args.action_scale,
            a_button_prev_state: false,
            jump_active: false,
            jump_phase: 0.0,
            jump_period: args.jump_period,
            jump_action_scale: args.jump_action_scale,
            jump_kp_ratio: args.jump_kp_ratio,
            x_button_prev_state: false,
            estimate_current: args.estimate_current,
            current_sum: 0.0,
            current_count: 0,
            current_peak: 0.0,
            roller_mode: args.roller,
            motorized_wheel_mode: args.motorized_wheel,
            has_standing_policy: args.standing.is_some(),
            is_using_standing: false,
            standing_kp_ratio: args.standing_kp_ratio,
            standing_prev_action_scale: args.action_scale,
            select_held_since: None,
            battery_benchmark: args.battery_benchmark.is_some(),
            benchmark_vel: args.benchmark_vel,
            benchmark_log,
            benchmark_recovering: false,
            benchmark_start_unix: 0.0,
            head_low_pass: args.head_low_pass,
            head_low_pass_alpha: args.head_low_pass_alpha,
            head_low_pass_prev: {
                let mut p = DEFAULT_POSITION;
                p[5] = args.neck_pitch_default;
                p[6] = args.head_pitch_default;
                [p[5], p[6], p[7], p[8]]
            },
            legs_low_pass: args.legs_low_pass,
            legs_low_pass_alpha: args.legs_low_pass_alpha,
            legs_low_pass_prev: {
                let p = DEFAULT_POSITION;
                [p[0], p[1], p[2], p[3], p[4], p[10], p[11], p[12], p[13], p[14]]
            },
            stream_listener: if args.stream {
                let addr = format!("0.0.0.0:{}", args.stream_port);
                let listener = std::net::TcpListener::bind(&addr)
                    .with_context(|| format!("Failed to bind stream listener on {}", addr))?;
                listener.set_nonblocking(true)
                    .context("Failed to set stream listener non-blocking")?;
                println!("Digital twin stream listening on TCP port {}", args.stream_port);
                Some(listener)
            } else {
                None
            },
            stream_client: None,
            odometry_engine: if args.stream {
                let path = args.urdf.clone().unwrap_or_else(|| {
                    std::env::var("HOME")
                        .map(|h| format!("{}/microduck/robot.urdf", h))
                        .unwrap_or_else(|_| "robot.urdf".to_string())
                });
                match odometry::Odometry::new(&path) {
                    Ok(odo) => {
                        println!("Odometry initialized from {}", path);
                        Some(odo)
                    }
                    Err(e) => {
                        eprintln!("Warning: odometry disabled — could not load URDF '{}': {}", path, e);
                        None
                    }
                }
            } else {
                None
            },
            odometry: [0.0; 4],
        })
    }

    /// Initialize motors (enable torque, set PID gains, set to initial positions)
    fn initialize_motors(&mut self) -> Result<()> {
        println!("Initializing motors...");

        // Check and correct EEPROM configuration (torque must be off)
        self.motor_controller.check_and_fix_config()
            .context("Failed to check motor configuration")?;

        // Set PID gains
        let (kp, ki, kd) = self.pid_gains;
        println!("Setting PID gains: kP={}, kI={}, kD={}", kp, ki, kd);
        self.motor_controller.set_pid_gains(kp, ki, kd)
            .context("Failed to set PID gains")?;

        // Enable torque on all motors
        self.motor_controller.set_torque_enable(true)
            .context("Failed to enable motor torque")?;

        // Motorized wheel mode: switch ankle motors to velocity control (Op Mode 1)
        if self.motorized_wheel_mode {
            self.motor_controller.set_wheel_motors_velocity_mode()
                .context("Failed to set wheel motors to velocity mode")?;
            println!("✓ Wheel motors set to velocity control (Operating Mode 1)");
        }

        // Smoothly interpolate to default position over 3 seconds
        println!("Moving to default position over 1 second...");
        self.motor_controller.interpolate_to_default(Duration::from_secs(1))
            .context("Failed to interpolate to default position")?;

        println!("✓ Motors initialized and moved to default position");

        // Read voltage now (motors settled): initialise EMA and capture recording metadata.
        // Always done so voltage_ema starts from a real reading even without --voltage-adapt.
        match self.motor_controller.read_voltages() {
            Ok(voltages) => {
                let avg = voltages.iter().map(|&v| v as f64).sum::<f64>() / voltages.len() as f64;
                self.voltage_ema = avg;
                if self.record_file.is_some() {
                    self.record_voltage = avg;
                }
                if self.voltage_adapt {
                    let eff = self.action_scale * (self.voltage_nominal / avg.clamp(6.0, 9.5));
                    println!("✓ Voltage adapt ON: measured={:.2}V, nominal={:.1}V → effective scale={:.3}",
                             avg, self.voltage_nominal, eff);
                } else if self.record_file.is_some() {
                    println!("✓ Recording metadata: action_scale={:.3}, voltage={:.2}V",
                             self.action_scale, avg);
                }
            }
            Err(e) => {
                println!("⚠ Could not read voltage (using nominal {:.1}V as EMA seed): {}", self.voltage_nominal, e);
            }
        }

        Ok(())
    }

    /// Run one control loop iteration
    fn control_step(&mut self) -> Result<()> {
        // Update controller input
        {
            self.controller.update()
                .context("Failed to update controller")?;

            if self.battery_benchmark {
                // Benchmark mode: only process emergency stop; ignore all stick/button input
                let select_pressed = self.controller.is_button_pressed("Select");
                if select_pressed {
                    let held_since = self.select_held_since.get_or_insert_with(Instant::now);
                    if held_since.elapsed() >= Duration::from_secs(2) {
                        println!("🛑 Emergency stop! Disabling motors and exiting (systemd will restart)...");
                        self.motor_controller.set_torque_enable(false).ok();
                        std::process::exit(1);
                    }
                } else {
                    self.select_held_since = None;
                }
                // Set command: always forward at configured velocity
                self.command = [self.benchmark_vel, 0.0, 0.0];
            } else {

            let state = self.controller.get_state();

            // Apply deadzone to all stick inputs
            let left_y = Controller::apply_deadzone(state.left_stick_y, self.controller_deadzone);
            let left_x = Controller::apply_deadzone(state.left_stick_x, self.controller_deadzone);
            let right_x = Controller::apply_deadzone(state.right_stick_x, self.controller_deadzone);
            let right_y = Controller::apply_deadzone(state.right_stick_y, self.controller_deadzone);

            // Handle Y button (West) to toggle head mode
            let y_pressed = self.controller.is_button_pressed("West");
            if y_pressed && !self.y_button_prev_state {
                self.head_mode = !self.head_mode;
                if self.head_mode {
                    self.command = [0.0; 3];
                    println!("HEAD mode: ON (L-stick: pitch/yaw, R-stick: roll/neck_pitch)");
                } else {
                    println!("HEAD mode: OFF");
                }
            }
            self.y_button_prev_state = y_pressed;

            // Handle B button (East) to toggle body pose mode
            let b_pressed = self.controller.is_button_pressed("East");
            if b_pressed && !self.b_button_prev_state {
                self.body_pose_mode = !self.body_pose_mode;
                if self.body_pose_mode {
                    self.body_cmd = [0.0; 3];
                    println!("BODY POSE mode: ON (L-stick up/down: z ±25mm, R-stick up: pitch ±20°, R-stick down: roll ±20°)");
                } else {
                    self.body_cmd = [0.0; 3];
                    println!("BODY POSE mode: OFF");
                }
            }
            self.b_button_prev_state = b_pressed;

            // Handle X button (North) to trigger one jump cycle
            let x_pressed = self.controller.is_button_pressed("North");
            if x_pressed && !self.x_button_prev_state && self.policy.has_jump() && !self.jump_active && !self.ground_pick_active {
                self.jump_active = true;
                self.jump_phase = 0.0;
                self.policy.set_jump_active(true);
                self.prev_action_scale = self.action_scale;
                self.action_scale = self.jump_action_scale;
                let (kp, ki, kd) = self.pid_gains;
                let jump_kp = (kp as f64 * self.jump_kp_ratio).round() as u16;
                self.motor_controller.set_pid_gains(jump_kp, ki, kd)
                    .context("Failed to set jump PID gains")?;
                println!("↑ Jump: started (period={:.1}s, action_scale={:.2}, kP={} → {})", self.jump_period, self.action_scale, kp, jump_kp);
            }
            self.x_button_prev_state = x_pressed;

            // Handle A button (South) to trigger one ground pick cycle
            let a_pressed = self.controller.is_button_pressed("South");
            if a_pressed && !self.a_button_prev_state && self.policy.has_ground_pick() && !self.ground_pick_active {
                self.ground_pick_active = true;
                self.ground_pick_phase = 0.0;
                self.policy.set_ground_pick_active(true);
                self.prev_action_scale = self.action_scale;
                self.action_scale = self.ground_pick_action_scale;
                let (kp, ki, kd) = self.pid_gains;
                let gp_kp = (kp as f64 * self.ground_pick_kp_ratio).round() as u16;
                self.motor_controller.set_pid_gains(gp_kp, ki, kd)
                    .context("Failed to set ground pick PID gains")?;
                println!("▼ Ground pick: started (period={:.1}s, action_scale={:.2}, kP={} → {})", self.ground_pick_period, self.action_scale, kp, gp_kp);
            }
            self.a_button_prev_state = a_pressed;

            if self.head_mode {
                // Head mode: joysticks control neck/head joint offsets
                // [neck_pitch, head_pitch, head_yaw, head_roll] → motor indices [5, 6, 7, 8]
                // EMA smoothing: offset += alpha * (target - offset)
                let targets = [
                    right_x as f64 * self.head_max,  // neck_pitch (R stick up/down)
                    left_x as f64 * self.head_max,   // head_pitch (L stick up/down)
                    left_y as f64 * self.head_max,   // head_yaw   (L stick left/right)
                    right_y as f64 * self.head_max,  // head_roll  (R stick left/right)
                ];
                for i in 0..4 {
                    self.head_offsets[i] += self.head_alpha * (targets[i] - self.head_offsets[i]);
                }
                self.command = [0.0; 3];
            } else if self.body_pose_mode {
                // Body pose mode: joysticks control standing body pose
                // - Left stick up/down (left_x)  → z height  (±25 mm)
                // - Right stick up/down (right_x) → pitch     (±30°)
                // - Right stick left/right (right_y) → roll   (±30°)
                const BODY_MAX_Z: f64 = 0.03;
                const BODY_MAX_ANGLE: f64 = 0.5236; // ~30° in radians
                let target_z     = left_x  as f64 * BODY_MAX_Z;
                let target_pitch = right_x as f64 * BODY_MAX_ANGLE;
                let target_roll  = right_y  as f64 * BODY_MAX_ANGLE;
                self.body_cmd[0] += self.cmd_alpha * (target_z     - self.body_cmd[0]);
                self.body_cmd[1] += self.cmd_alpha * (target_pitch  - self.body_cmd[1]);
                self.body_cmd[2] += self.cmd_alpha * (target_roll   - self.body_cmd[2]);
                self.command = [0.0; 3]; // keep command near zero to stay in standing mode
            } else if self.roller_mode {
                // Roller mode: asymmetric vel_x (0.6 push / -0.5 brake), no lateral, ±1.0 rad heading
                let vel_x_raw = left_x as f64;
                let vel_x = if vel_x_raw >= 0.0 { vel_x_raw * 0.6 } else { vel_x_raw * 0.5 };
                let target_cmd = [vel_x, 0.0, -right_y as f64 * 1.0];
                for i in 0..3 {
                    self.command[i] += self.cmd_alpha * (target_cmd[i] - self.command[i]);
                }
            } else if self.motorized_wheel_mode {
                // Motorized wheel (Segway): left stick forward/back = vel_x, right stick L/R = turn, no strafe
                let vel_x_raw = left_x as f64;
                let vel_x = if vel_x_raw >= 0.0 { vel_x_raw * self.max_linear_vel } else { vel_x_raw * self.max_linear_vel };
                let target_cmd = [vel_x, 0.0, -right_y as f64 * self.max_angular_vel];
                for i in 0..3 {
                    self.command[i] += self.cmd_alpha * (target_cmd[i] - self.command[i]);
                }
            } else {
                // Normal mode: joysticks control velocity commands
                // - Left stick X (up/down) → vel_x forward/backward
                // - Left stick Y (left/right) → vel_y strafe
                // - Right stick Y (left/right) → vel_z turn
                // EMA smoothing: cmd += alpha * (target - cmd)
                let target_cmd = [
                    left_x as f64 * self.max_linear_vel,
                    -left_y as f64 * self.max_linear_vel,
                    -right_y as f64 * 1.5 * self.max_angular_vel,
                ];
                for i in 0..3 {
                    self.command[i] += self.cmd_alpha * (target_cmd[i] - self.command[i]);
                }
            }

            // Right trigger controls mouth: offset (mouth mode) or absolute position (legacy mode)
            let mouth_range = MOUTH_MAX_ANGLE - MOUTH_MIN_ANGLE;
            if self.mouth_enabled {
                self.mouth_offset = state.right_trigger as f64 * mouth_range;
            } else {
                self.mouth_position = MOUTH_MIN_ANGLE + state.right_trigger as f64 * mouth_range;
            }

            // Handle Start button to toggle policy inference (or recover from fall)
            let start_pressed = self.controller.is_button_pressed("Start");
            if start_pressed && !self.start_button_prev_state {
                // Button was just pressed (rising edge)
                if self.fallen {
                    // Recover from fall: restore kP and re-enable policy
                    self.fallen = false;
                    self.fall_detected_since = None;
                    self.policy_enabled = true;
                    let (kp, ki, kd) = self.pid_gains;
                    self.motor_controller.set_pid_gains(kp, ki, kd)
                        .context("Failed to restore PID gains after fall")?;
                    println!("▶ Fall recovery: PID restored (kP={}), policy ENABLED", kp);
                } else {
                    self.policy_enabled = !self.policy_enabled;
                    if self.policy_enabled {
                        println!("▶ Policy inference ENABLED");
                    } else {
                        println!("⏸ Policy inference DISABLED - returning to default pose");
                    }
                }
            }
            self.start_button_prev_state = start_pressed;

            // Hold Select for 2s → emergency stop + restart from scratch
            let select_pressed = self.controller.is_button_pressed("Select");
            if select_pressed {
                let held_since = self.select_held_since.get_or_insert_with(Instant::now);
                if held_since.elapsed() >= Duration::from_secs(2) {
                    println!("🛑 Emergency stop! Disabling motors and exiting (systemd will restart)...");
                    self.motor_controller.set_torque_enable(false).ok();
                    std::process::exit(1);
                }
            } else {
                self.select_held_since = None;
            }
            } // end else (non-benchmark controller processing)
        }

        // Read IMU data
        let mut imu_data = self.imu_controller.read()
            .context("Failed to read IMU data")?;

        // Apply pitch offset if specified (rotation around Y axis)
        if self.pitch_offset != 0.0 {
            let cos_pitch = self.pitch_offset.cos();
            let sin_pitch = self.pitch_offset.sin();
            let x = imu_data.accel[0];
            let z = imu_data.accel[2];

            // Rotate projected gravity around Y axis
            imu_data.accel[0] = x * cos_pitch + z * sin_pitch;
            imu_data.accel[2] = -x * sin_pitch + z * cos_pitch;

            // Renormalize
            let mag = (imu_data.accel[0].powi(2) + imu_data.accel[1].powi(2) + imu_data.accel[2].powi(2)).sqrt();
            if mag > 0.01 {
                imu_data.accel[0] /= mag;
                imu_data.accel[1] /= mag;
                imu_data.accel[2] /= mag;
            }
        }

        // Fall detection: accel is projected gravity from quaternion (no walking dynamics).
        // When upright accel[2] ≈ -1.0; above -0.5 means >60° tilt.
        // Debounce: only trigger after 0.2s of continuous detection.
        if self.battery_benchmark {
            // Benchmark fall detection: auto-recover via standing policy (command → 0)
            if self.benchmark_recovering {
                if imu_data.accel[2] < -0.5 {
                    // Robot is upright again: resume benchmark walking
                    self.benchmark_recovering = false;
                    self.fall_detected_since = None;
                    println!("▶ Benchmark: upright, resuming walk");
                }
            } else if imu_data.accel[2] > -0.5 {
                let since = self.fall_detected_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(200) {
                    self.benchmark_recovering = true;
                    self.fall_detected_since = None;
                    println!("⚠ FALLEN (benchmark)! Switching to standing policy for recovery...");
                }
            } else {
                self.fall_detected_since = None;
            }
        } else if !self.has_standing_policy && !self.fallen {
            // Standard fall detection: skipped when a standing policy is provided.
            if imu_data.accel[2] > -0.5 {
                let since = self.fall_detected_since.get_or_insert_with(Instant::now);
                if since.elapsed() >= Duration::from_millis(200) {
                    self.fallen = true;
                    self.fall_detected_since = None;
                    self.policy_enabled = false;
                    self.motor_controller.set_pid_gains(50, self.pid_gains.1, self.pid_gains.2)
                        .context("Failed to set fall-limp PID gains")?;
                    println!("⚠ FALLEN! Motors limp (kP=50), policy stopped. Press Start to recover.");
                }
            } else {
                self.fall_detected_since = None;  // Reset timer if robot recovers before threshold
            }
        }

        // Read motor state
        let motor_state = self.motor_controller.read_state()
            .context("Failed to read motor state")?;

        // Digital twin streaming: send [qw, qx, qy, qz, j0..j14] as 19 × f32 LE
        if self.stream_listener.is_some() {
            // Accept a new client if we don't have one
            if self.stream_client.is_none() {
                if let Some(listener) = &self.stream_listener {
                    if let Ok((stream, addr)) = listener.accept() {
                        let _ = stream.set_nonblocking(true);
                        let _ = stream.set_nodelay(true);
                        println!("Digital twin client connected: {}", addr);
                        self.stream_client = Some(stream);
                    }
                }
            }
            // Send packet to connected client
            if let Some(client) = &mut self.stream_client {
                let q = imu_data.quat;
                let mut buf = [0u8; 34 * 4];
                let floats: [f32; 34] = [
                    // [0..3]   IMU quaternion [w, x, y, z]
                    q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32,
                    // [4..18]  joint positions (runtime motor order)
                    motor_state.positions[0] as f32,
                    motor_state.positions[1] as f32,
                    motor_state.positions[2] as f32,
                    motor_state.positions[3] as f32,
                    motor_state.positions[4] as f32,
                    motor_state.positions[5] as f32,
                    motor_state.positions[6] as f32,
                    motor_state.positions[7] as f32,
                    motor_state.positions[8] as f32,
                    motor_state.positions[9] as f32,
                    motor_state.positions[10] as f32,
                    motor_state.positions[11] as f32,
                    motor_state.positions[12] as f32,
                    motor_state.positions[13] as f32,
                    motor_state.positions[14] as f32,
                    // [19..33] motor currents in mA (same order)
                    motor_state.currents_ma[0] as f32,
                    motor_state.currents_ma[1] as f32,
                    motor_state.currents_ma[2] as f32,
                    motor_state.currents_ma[3] as f32,
                    motor_state.currents_ma[4] as f32,
                    motor_state.currents_ma[5] as f32,
                    motor_state.currents_ma[6] as f32,
                    motor_state.currents_ma[7] as f32,
                    motor_state.currents_ma[8] as f32,
                    motor_state.currents_ma[9] as f32,
                    motor_state.currents_ma[10] as f32,
                    motor_state.currents_ma[11] as f32,
                    motor_state.currents_ma[12] as f32,
                    motor_state.currents_ma[13] as f32,
                    motor_state.currents_ma[14] as f32,
                ];
                for (i, f) in floats.iter().enumerate() {
                    buf[i*4..(i+1)*4].copy_from_slice(&f.to_le_bytes());
                }
                if client.write_all(&buf).is_err() {
                    println!("Digital twin client disconnected");
                    self.stream_client = None;
                }
            }
        }

        // Inline odometry — runs whenever --stream is active and a URDF was loaded
        if let Some(odo) = &mut self.odometry_engine {
            let angles: [f64; 15] = std::array::from_fn(|i| motor_state.positions[i]);
            odo.update(&angles, imu_data.quat);
            let p = odo.position;
            self.odometry = [p[0], p[1], p[2], odo.yaw];
        }

        // Current estimation: accumulate running stats at every control step
        if self.estimate_current {
            let total: f64 = motor_state.currents_ma.iter().sum();
            self.current_sum += total;
            self.current_count += 1;
            if total > self.current_peak {
                self.current_peak = total;
            }
        }

        // Ground pick: override command with phase encoding [cos, sin, 0] and use 51D obs
        // Body pose mode: put normalized body_cmd into command slot (51D obs, matching standing policy training)
        let gp_cmd;
        let jump_cmd;
        let standing_obs_cmd;
        let (effective_command, obs_command) = if self.ground_pick_active {
            let angle = 2.0 * std::f64::consts::PI * self.ground_pick_phase;
            gp_cmd = [angle.cos(), angle.sin(), 0.0];
            (&gp_cmd, &gp_cmd)
        } else if self.jump_active {
            let angle = 2.0 * std::f64::consts::PI * self.jump_phase;
            jump_cmd = [angle.cos(), angle.sin(), 0.0];
            (&jump_cmd, &jump_cmd)
        } else if self.body_pose_mode {
            // Keep effective_command = [0,0,0] so policy stays in standing mode.
            // Pass normalized body_cmd as the obs command (replaces vel cmd in 51D standing obs).
            standing_obs_cmd = [
                self.body_cmd[0] / 0.03,
                self.body_cmd[1] / 0.5236,
                self.body_cmd[2] / 0.5236,
            ];
            (&self.command, &standing_obs_cmd)
        } else {
            (&self.command, &self.command)
        };

        // Standing policy transitions: apply action_scale=1 and kp=60% when switching to standing
        if !self.ground_pick_active && !self.jump_active && !self.roller_mode && !self.motorized_wheel_mode {
            let will_stand = self.policy.will_use_standing(effective_command);
            if will_stand && !self.is_using_standing {
                self.is_using_standing = true;
                self.standing_prev_action_scale = self.action_scale;
                self.action_scale = 1.0;
                let (kp, ki, kd) = self.pid_gains;
                let standing_kp = (kp as f64 * self.standing_kp_ratio).round() as u16;
                self.motor_controller.set_pid_gains(standing_kp, ki, kd)
                    .context("Failed to set standing PID gains")?;
                println!("^ Standing mode: kP={} -> {} ({:.0}%), action_scale=1.0", kp, standing_kp, self.standing_kp_ratio * 100.0);
            } else if !will_stand && self.is_using_standing {
                self.is_using_standing = false;
                self.action_scale = self.standing_prev_action_scale;
                let (kp, ki, kd) = self.pid_gains;
                self.motor_controller.set_pid_gains(kp, ki, kd)
                    .context("Failed to restore PID gains after standing")?;
                println!("v Walking mode: kP restored to {}, action_scale={:.2}", kp, self.action_scale);
            }
        }

        let observation = if self.motorized_wheel_mode {
            Observation::new_motorized_wheel(
                &imu_data,
                obs_command,
                &motor_state,
                &self.last_action,
                &self.default_positions,
            )
        } else {
            Observation::new(
                &imu_data,
                obs_command,
                &motor_state,
                &self.last_action,
                &self.default_positions,
                self.mouth_enabled,
            )
        };

        // Run policy inference (or hold default position if policy disabled)
        let action = if self.policy_enabled {
            self.policy.infer(&observation, effective_command, self.mouth_enabled)
                .context("Failed to run policy inference")?
        } else {
            // During standby, use zero actions (hold default position)
            [0.0f32; NUM_MOTORS]
        };

        // Convert action offsets to motor targets: init_pos + effective_action_scale * action.
        // When voltage_adapt is on, scale proportionally to battery voltage so that the
        // effective kP (which scales linearly with supply voltage) stays at the nominal level.
        let effective_action_scale = if self.voltage_adapt {
            // Clamp EMA to [6.0, 9.5] V to guard against bad readings or extreme states.
            self.action_scale * (self.voltage_nominal / self.voltage_ema.clamp(6.0, 9.5))
        } else {
            self.action_scale
        };
        let mut motor_targets = [0.0f64; NUM_MOTORS];
        for i in 0..NUM_MOTORS {
            motor_targets[i] = self.default_positions[i] + effective_action_scale * action[i] as f64;
        }

        // Apply head offsets to neck/head joints (motor indices 5-8: neck_pitch, head_pitch, head_yaw, head_roll)
        for i in 0..4 {
            motor_targets[5 + i] += self.head_offsets[i];
        }

        // Mouth motor (index 9): policy + trigger offset in mouth mode, trigger absolute in legacy mode
        if self.mouth_enabled {
            motor_targets[MOUTH_MOTOR_IDX] += self.mouth_offset;
        } else {
            motor_targets[MOUTH_MOTOR_IDX] = self.mouth_position;
        }

        // Low-pass filter for head/neck joints to suppress oscillation on lightly-loaded joints
        if self.head_low_pass {
            let alpha = self.head_low_pass_alpha;
            for i in 0..4 {
                let filtered = alpha * motor_targets[5 + i] + (1.0 - alpha) * self.head_low_pass_prev[i];
                self.head_low_pass_prev[i] = filtered;
                motor_targets[5 + i] = filtered;
            }
        }

        // Low-pass filter for leg joints (left: 0-4, right: 10-14)
        if self.legs_low_pass {
            let alpha = self.legs_low_pass_alpha;
            for i in 0..5 {
                let filtered = alpha * motor_targets[i] + (1.0 - alpha) * self.legs_low_pass_prev[i];
                self.legs_low_pass_prev[i] = filtered;
                motor_targets[i] = filtered;
            }
            for i in 0..5 {
                let filtered = alpha * motor_targets[10 + i] + (1.0 - alpha) * self.legs_low_pass_prev[5 + i];
                self.legs_low_pass_prev[5 + i] = filtered;
                motor_targets[10 + i] = filtered;
            }
        }

        // Get timestamp for both logging and recording
        let timestamp = if let Some(start) = self.start_time {
            start.elapsed().as_secs_f64()
        } else {
            0.0
        };

        // Record observation if recording mode is enabled
        if self.record_file.is_some() {
            let obs_vec = observation.as_slice().to_vec();
            self.recorded_observations.push(TimestampedObservation {
                timestamp,
                observation: obs_vec,
            });
        }

        // Log data if CSV logging is enabled
        if let Some(ref mut file) = self.log_file {
            // Write step and time
            write!(file, "{},{:.4}", self.step_counter, timestamp)?;

            // Write all observations (size varies: 51 or 53)
            let obs_slice = observation.as_slice();
            for val in obs_slice {
                write!(file, ",{:.6}", val)?;
            }

            // Write all 14 actions
            for i in 0..NUM_MOTORS {
                write!(file, ",{:.6}", action[i])?;
            }

            writeln!(file)?;

            self.step_counter += 1;
        }

        if self.motorized_wheel_mode {
            // Motorized wheel: action at WHEEL_MOTOR_INDICES are wheel velocity outputs.
            // The legacy 14→15 mapping preserves joint ordering: action[4]=left_wheel_vel (policy[4]),
            // action[14]=right_wheel_vel (policy[13]). motor_targets at those indices are ignored
            // because write_goal_positions_no_wheels skips them.
            let left_vel  = action[WHEEL_MOTOR_INDICES[0]] as f64 * WHEEL_MAX_VEL;
            let right_vel = action[WHEEL_MOTOR_INDICES[1]] as f64 * WHEEL_MAX_VEL;
            self.motor_controller.write_wheel_velocities(left_vel, right_vel)
                .context("Failed to write wheel velocities")?;
            self.motor_controller.write_goal_positions_no_wheels(&motor_targets)
                .context("Failed to write motor positions (no wheels)")?;
        } else {
            self.motor_controller.write_goal_positions(&motor_targets)
                .context("Failed to write motor positions")?;
        }

        // Update last action
        self.last_action = action;

        // Advance ground pick phase; end cycle when complete
        if self.ground_pick_active {
            let dt = 1.0 / self.control_freq as f64;
            self.ground_pick_phase += dt / self.ground_pick_period;
            if self.ground_pick_phase >= 0.7 {
                self.ground_pick_active = false;
                self.ground_pick_phase = 0.0;
                self.policy.set_ground_pick_active(false);
                self.action_scale = self.prev_action_scale;
                let (kp, ki, kd) = self.pid_gains;
                if self.is_using_standing {
                    // Return to standing mode: re-apply 60% kP (transition won't fire since
                    // is_using_standing is already true, so we must restore it explicitly)
                    let standing_kp = (kp as f64 * self.standing_kp_ratio).round() as u16;
                    self.motor_controller.set_pid_gains(standing_kp, ki, kd)
                        .context("Failed to restore standing PID gains after ground pick")?;
                    println!("▲ Ground pick: done, returning to standing (action_scale={:.2}, kP={})", self.action_scale, standing_kp);
                } else {
                    self.motor_controller.set_pid_gains(kp, ki, kd)
                        .context("Failed to restore PID gains after ground pick")?;
                    println!("▲ Ground pick: done, returning to walking (action_scale={:.2}, kP restored to {})", self.action_scale, kp);
                }
            }
        }

        // Advance jump phase; one-shot: clamp at 1.0 then hand back to walking/standing
        if self.jump_active {
            let dt = 1.0 / self.control_freq as f64;
            self.jump_phase = (self.jump_phase + dt / self.jump_period).min(1.0);
            if self.jump_phase >= 1.0 {
                self.jump_active = false;
                self.jump_phase = 0.0;
                self.policy.set_jump_active(false);
                self.action_scale = self.prev_action_scale;
                let (kp, ki, kd) = self.pid_gains;
                if self.is_using_standing {
                    let standing_kp = (kp as f64 * self.standing_kp_ratio).round() as u16;
                    self.motor_controller.set_pid_gains(standing_kp, ki, kd)
                        .context("Failed to restore standing PID gains after jump")?;
                    println!("↓ Jump: done, returning to standing (action_scale={:.2}, kP={})", self.action_scale, standing_kp);
                } else {
                    self.motor_controller.set_pid_gains(kp, ki, kd)
                        .context("Failed to restore PID gains after jump")?;
                    println!("↓ Jump: done, returning to walking (action_scale={:.2}, kP restored to {})", self.action_scale, kp);
                }
            }
        }

        Ok(())
    }

    /// Run the main control loop
    async fn run(&mut self, shutdown_flag: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Result<()> {
        println!("Starting control loop at {} Hz (using tokio interval)", self.control_freq);

        // Set start time for CSV logging and recording
        self.start_time = Some(Instant::now());
        let runtime_start = Instant::now();

        if self.battery_benchmark {
            self.policy_enabled = true;
            self.benchmark_start_unix = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs_f64();
            println!("✓ Battery benchmark started: policy auto-enabled");
        }

        // In recording mode, schedule policy to enable after 1 second standby
        let policy_enable_time = if !self.policy_enabled && self.record_file.is_some() {
            Some(Instant::now() + Duration::from_secs(1))
        } else {
            None
        };

        let dt = Duration::from_secs_f64(1.0 / self.control_freq as f64);
        let mut iteration: u64 = 0;
        let mut total_time = Duration::ZERO;
        let mut min_loop_time = Duration::from_secs(999999);
        let mut max_loop_time = Duration::ZERO;
        let mut last_report_iteration: u64 = 0;
        let mut missed_deadlines: u64 = 0;

        // Create tokio interval with MissedTickBehavior::Burst
        // Burst mode will catch up on missed ticks rather than skipping them
        let mut interval = tokio::time::interval(dt);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Burst);

        while !shutdown_flag.load(std::sync::atomic::Ordering::SeqCst) {
            // Wait for next tick
            interval.tick().await;

            let loop_start = Instant::now();

            // Enable policy after 1 second in recording mode
            if let Some(enable_time) = policy_enable_time {
                if loop_start >= enable_time && !self.policy_enabled {
                    self.policy_enabled = true;
                    println!("✓ Policy inference enabled (after 1s standby)");
                }
            }

            // Run control step
            if let Err(e) = self.control_step() {
                eprintln!("Error in control step: {}", e);
                // Continue running despite errors
            }

            // Calculate timing
            let elapsed = loop_start.elapsed();
            total_time += elapsed;
            iteration += 1;

            // Track min/max for jitter calculation
            if elapsed < min_loop_time {
                min_loop_time = elapsed;
            }
            if elapsed > max_loop_time {
                max_loop_time = elapsed;
            }

            // Print statistics every second
            if iteration % self.control_freq as u64 == 0 {
                let avg_time = total_time / iteration as u32;
                let jitter = max_loop_time.as_secs_f64() - min_loop_time.as_secs_f64();
                let total_elapsed = runtime_start.elapsed().as_secs_f64();

                // Calculate actual frequency achieved
                let actual_freq = if total_elapsed > 0.0 {
                    iteration as f64 / total_elapsed
                } else {
                    0.0
                };

                // Read leg currents (for display only)
                let currents_result = self.motor_controller.read_leg_currents();
                let current_str = match currents_result {
                    Ok((left, right, _total)) => format!(
                        "Left: {:.0} mA, Right: {:.0} mA, Total: {:.0} mA",
                        left, right, left + right),
                    Err(_) => "Current read failed".to_string(),
                };

                let cmd_str = format!(" | Cmd: [{:.2}, {:.2}, {:.2}]",
                    self.command[0], self.command[1], self.command[2]);

                println!(
                    "⏱️  Runtime: {:.1}s | Iter: {} | Avg: {:.2}ms ({:.1}%) | Min: {:.2}ms | Max: {:.2}ms | Jitter: {:.2}ms | Freq: {:.1}Hz | {}{}",
                    total_elapsed,
                    iteration,
                    avg_time.as_secs_f64() * 1000.0,
                    (avg_time.as_secs_f64() / dt.as_secs_f64()) * 100.0,
                    min_loop_time.as_secs_f64() * 1000.0,
                    max_loop_time.as_secs_f64() * 1000.0,
                    jitter * 1000.0,
                    actual_freq,
                    current_str,
                    cmd_str
                );

                // Reset jitter tracking for next interval
                min_loop_time = Duration::from_secs(999999);
                max_loop_time = Duration::ZERO;
                last_report_iteration = iteration;

                // Flush log file every second
                if let Some(ref mut file) = self.log_file {
                    file.flush().ok();
                }

                // Read voltage once per second: update EMA and/or battery benchmark log.
                // EMA is only updated when --voltage-adapt is active; benchmark always needs it.
                if self.voltage_adapt || self.battery_benchmark {
                    let avg_voltage = self.motor_controller.read_voltages()
                        .map(|v| v.iter().map(|&x| x as f64).sum::<f64>() / v.len() as f64)
                        .unwrap_or(0.0);

                    if self.voltage_adapt && avg_voltage > 0.0 {
                        // Slow EMA: α=0.1 → ~10 s time constant at 1 sample/s.
                        // Smooths over per-step load sags while tracking battery depletion.
                        const VOLTAGE_EMA_ALPHA: f64 = 0.1;
                        self.voltage_ema = VOLTAGE_EMA_ALPHA * avg_voltage
                            + (1.0 - VOLTAGE_EMA_ALPHA) * self.voltage_ema;
                    }

                    if self.battery_benchmark {
                        println!("🔋 Benchmark elapsed: {:.0}s  avg voltage: {:.2}V", total_elapsed, avg_voltage);
                        if let Some(ref mut file) = self.benchmark_log {
                            let current_unix = SystemTime::now()
                                .duration_since(UNIX_EPOCH)
                                .unwrap_or_default()
                                .as_secs_f64();
                            let elapsed = current_unix - self.benchmark_start_unix;
                            writeln!(file, "{:.3},{:.3},{:.3},{:.3}", self.benchmark_start_unix, elapsed, current_unix, avg_voltage).ok();
                            file.flush().ok();
                        }
                    }
                }
            }

            // Check if we overran (tokio interval will handle timing automatically)
            if elapsed > dt {
                missed_deadlines += 1;
                if missed_deadlines % 10 == 1 {  // Print every 10th miss
                    eprintln!(
                        "Warning: Missed deadline #{} - Target: {:.2} ms, Actual: {:.2} ms (overrun: {:.2} ms)",
                        missed_deadlines,
                        dt.as_secs_f64() * 1000.0,
                        elapsed.as_secs_f64() * 1000.0,
                        (elapsed.as_secs_f64() - dt.as_secs_f64()) * 1000.0
                    );
                }
            }
        }

        Ok(())
    }

    /// Block until the Start button is pressed (rising edge).
    /// If `blink` is true, pulses all motor LEDs in a double-blink heartbeat
    /// pattern (ON 100ms · OFF 100ms · ON 100ms · OFF 700ms) so the user
    /// can tell the robot is ready and waiting — distinct from a motor error LED.
    async fn wait_for_start_button(&mut self, blink: bool) -> Result<()> {
        // Drain any currently-pressed state first so we wait for a fresh press
        loop {
            self.controller.update().context("Failed to update controller")?;
            if !self.controller.is_button_pressed("Start") {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        // Now wait for a rising edge, blinking LEDs if requested
        // Pattern (20 ms ticks, period = 50 ticks = 1 000 ms):
        //   ticks  0- 4 → ON   (100 ms)
        //   ticks  5- 9 → OFF  (100 ms)
        //   ticks 10-14 → ON   (100 ms)
        //   ticks 15-49 → OFF  (700 ms)
        let mut tick: u64 = 0;
        loop {
            self.controller.update().context("Failed to update controller")?;
            if self.controller.is_button_pressed("Start") {
                if blink {
                    self.motor_controller.set_all_leds(false).ok();
                }
                return Ok(());
            }
            if blink {
                let phase = tick % 50;
                match phase {
                    0  => { self.motor_controller.set_all_leds(true).ok(); }
                    5  => { self.motor_controller.set_all_leds(false).ok(); }
                    10 => { self.motor_controller.set_all_leds(true).ok(); }
                    15 => { self.motor_controller.set_all_leds(false).ok(); }
                    _  => {}
                }
            }
            tick += 1;
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    /// Shutdown the runtime safely
    fn shutdown(&mut self) -> Result<()> {
        println!("Shutting down runtime...");

        // Save recorded observations if recording mode is enabled
        if let Some(ref path) = self.record_file {
            println!("Saving {} recorded observations to {}...",
                     self.recorded_observations.len(), path);

            let unix_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs_f64())
                .unwrap_or(0.0);

            // Use voltage_ema if it was updated during the session (voltage_adapt or always-read),
            // otherwise fall back to the startup snapshot (record_voltage).
            let recorded_voltage = if self.voltage_ema != self.voltage_nominal {
                self.voltage_ema
            } else {
                self.record_voltage
            };
            let recording = RecordingData {
                metadata: RecordingMetadata {
                    action_scale: self.action_scale,  // base scale; sysid computes effective = base * (7.4 / voltage_v)
                    voltage_v:    recorded_voltage,
                    freq_hz:      self.control_freq,
                    unix_time,
                },
                observations: self.recorded_observations.clone(),
            };

            let mut file = File::create(path)
                .context(format!("Failed to create recording file: {}", path))?;

            serde_pickle::to_writer(&mut file, &recording, Default::default())
                .context("Failed to serialize observations to pickle format")?;

            println!("✓ Recorded observations saved to {} (action_scale={:.3}, voltage={:.2}V)",
                     path, self.action_scale, recorded_voltage);
        }

        // Flush and close log file if it exists
        if let Some(ref mut file) = self.log_file {
            file.flush()?;
            println!("✓ Log file flushed and closed");
        }

        // Print current estimation results if enabled
        if self.estimate_current && self.current_count > 0 {
            let mean = self.current_sum / self.current_count as f64;
            println!("\n=== Current Estimation ({} samples at control freq) ===", self.current_count);
            println!("  Mean total current : {:.0} mA", mean);
            println!("  Peak total current : {:.0} mA", self.current_peak);
            println!("  (All 14 body motors cumulated)");
        }

        // Motorized wheel: restore ankle motors to position control so robot can be reused normally
        if self.motorized_wheel_mode {
            match self.motor_controller.set_wheel_motors_position_mode() {
                Ok(()) => println!("✓ Wheel motors restored to position control (Operating Mode {})", OPERATING_MODE_POSITION),
                Err(e) => eprintln!("⚠ Failed to restore wheel motors to position mode: {}", e),
            }
        }

        // Note: We do NOT disable motor torque on shutdown
        // Motors will maintain their last commanded position
        // To manually disable torque, use a separate command or power off the motors

        println!("✓ Runtime shutdown complete (motors remain enabled)");
        Ok(())
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    println!("=== Microduck Robot Runtime ===\n");

    // Parse command line arguments
    let mut args = Args::parse();

    // Expand ~ in path arguments (clap default_value doesn't get shell expansion)
    let home = std::env::var("HOME").unwrap_or_default();
    let expand = |p: Option<String>| p.and_then(|s| {
        if s.eq_ignore_ascii_case("none") { None }
        else if s.starts_with("~/") { Some(format!("{}/{}", home, &s[2..])) }
        else { Some(s) }
    });
    args.model = expand(args.model);
    args.standing = expand(args.standing);
    args.ground_pick = expand(args.ground_pick);

    // Create runtime
    let mut runtime = Runtime::new(&args)?;

    // Set up Ctrl+C handler for graceful shutdown
    let shutdown_flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let shutdown_flag_clone = shutdown_flag.clone();

    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, shutting down...");
        shutdown_flag_clone.store(true, std::sync::atomic::Ordering::SeqCst);
    })
    .context("Failed to set Ctrl+C handler")?;

    if args.battery_benchmark.is_some() {
        // Battery benchmark: no controller needed, initialize and go immediately
        println!("\n⚡ Battery benchmark: initializing motors and starting immediately...");
        runtime.initialize_motors()?;
    } else {
        // Step 1: wait for first Start press → initialize motors
        println!("\n⏳ Press Start to initialize motors...");
        runtime.wait_for_start_button(true).await?;
        runtime.initialize_motors()?;

        // If recording mode is enabled, wait 1 second before starting the control loop
        // This gives the robot time to settle, then we'll record 1 second of standby
        if args.record.is_some() {
            println!("Recording mode: waiting 1 second before starting control loop...");
            tokio::time::sleep(Duration::from_secs(1)).await;
            println!("✓ Starting control loop (1s standby recording, then policy inference)");
        }

        // Step 2: wait for second Start press → run the policy
        println!("\n⏳ Press Start to run the policy...");
        runtime.wait_for_start_button(false).await?;
    }

    // Run main loop (will exit when Ctrl+C is pressed)
    runtime.run(shutdown_flag).await?;

    // Clean shutdown
    runtime.shutdown()?;

    Ok(())
}
