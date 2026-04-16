mod imu;
mod motor;
mod observation;
mod policy;
mod controller;
mod odometry;
mod camera;

use anyhow::{Context, Result};
use clap::Parser;
use imu::{ImuController, Bno08xController, Bmi088Controller, AnyImuController};
use motor::{MotorController, NUM_MOTORS, DEFAULT_POSITION, FOLD_DEFAULT_POSITION, MOUTH_MOTOR_IDX, MOUTH_MIN_ANGLE, MOUTH_MAX_ANGLE,
            DEFAULT_POSITION_MOTORIZED_WHEEL, WHEEL_MOTOR_INDICES, WHEEL_MAX_VEL, OPERATING_MODE_POSITION};
use observation::Observation;
use policy::Policy;
use controller::Controller;
use serde::Serialize;
use std::fs::File;
use std::io::{Read as IoRead, Write as IoWrite};
use std::net::UdpSocket;
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

    /// Control loop frequency in Hz (policy inference + motor I/O)
    #[arg(short, long, default_value_t = 50)]
    freq: u32,

    /// Odometry update frequency in Hz (must be a multiple of --freq).
    /// Extra ticks read the IMU and update odometry with cached joint angles,
    /// giving smoother position/yaw estimates without affecting policy timing.
    #[arg(long, default_value_t = 100)]
    odo_freq: u32,

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

    /// Use BNO08X IMU (BNO080/085/086) instead of the default BMI088
    /// Connects via I2C at address 0x4A (default) on /dev/i2c-1
    #[arg(long)]
    bno08x: bool,

    /// Use BNO055 IMU instead of the default BMI088
    /// Connects via I2C on /dev/i2c-1 at address 0x28
    #[arg(long)]
    bno055: bool,

    /// Use BMI088 IMU (default, explicit selection)
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

    /// Wheel velocity scale in motorized-wheel mode (multiplies WHEEL_MAX_VEL).
    /// Position joints always use --action-scale. Use this to tune wheel authority
    /// independently, e.g. 1.2 for more aggressive balancing or 0.8 to reduce
    /// wheel speed.
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    motorized_wheel_action_scale: f64,

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

    /// Fold robot mode: use fold robot STAND default pose, continuous fold/unfold phase cycling.
    /// B button toggles fold cycling. Command becomes [cos(2π·φ), sin(2π·φ), 0].
    #[arg(long)]
    fold: bool,

    /// Path to fold/unfold policy ONNX file (required with --fold)
    #[arg(long)]
    fold_policy: Option<String>,

    /// Duration of one fold/unfold cycle in seconds (default: 6.0)
    #[arg(long, default_value_t = 6.0)]
    fold_period: f64,

    /// Action scale for fold policy (default: 1.0)
    #[arg(long, default_value_t = 1.0)]
    fold_action_scale: f64,

    /// Motorized wheel mode: ankle motors use velocity control (Operating Mode 1).
    /// Observation is 49D (no wheel joint_pos, no mouth). Never switches to standing policy.
    #[arg(long)]
    motorized_wheel: bool,

    /// Invert wheel velocity directions (negate both left and right wheel commands).
    /// Use this if the robot moves backwards or spins when it should go straight,
    /// to compensate for physical motor mounting orientation vs. simulation convention.
    #[arg(long)]
    invert_wheel: bool,

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

    /// Stream robot state over TCP for digital twin visualization.
    /// Sends 39 × f32 LE (156 bytes/frame):
    ///   [0..3]   IMU quaternion [w, x, y, z]
    ///   [4..18]  joint positions (runtime motor order)
    ///   [19..33] motor currents in mA
    ///   [34..35] odometry x, y in metres
    ///   [36..38] ball world position [x, y, z] in metres (NaN if not detected recently)
    #[arg(long)]
    stream: bool,

    /// TCP port for digital twin streaming (default: 9870)
    #[arg(long, default_value_t = 9870)]
    stream_port: u16,

    /// TCP port for JPEG camera stream sent to the brain (default: 9871, requires --stream)
    #[arg(long, default_value_t = 9871)]
    jpeg_port: u16,

    /// UDP port for velocity commands received from the brain (default: 9872, requires --stream)
    #[arg(long, default_value_t = 9872)]
    cmd_port: u16,

    /// Camera capture width in pixels (default: 640)
    #[arg(long, default_value_t = 640u32)]
    cam_width: u32,

    /// Camera capture height in pixels (default: 480)
    #[arg(long, default_value_t = 480u32)]
    cam_height: u32,

    /// Camera capture framerate (default: 5 Hz)
    #[arg(long, default_value_t = 5u32)]
    cam_fps: u32,

    /// Enable IMX500 onboard object detection (requires --stream).
    /// Passes --post-process-file to rpicam-vid to run the SSD MobileNetV2
    /// COCO model on-sensor; streams bounding-box JSON to --ball-detect-port.
    /// Requires: sudo apt install imx500-all
    #[arg(long)]
    ball_detect: bool,

    /// TCP port for ball-detection JSON stream (default: 9873, requires --ball-detect)
    #[arg(long, default_value_t = 9873)]
    ball_detect_port: u16,

    /// Proportional gain for ball-tracking head control.
    /// -1.0 (default) = head points directly at the ball; reduce magnitude if oscillatory.
    /// Flip the sign if the head moves in the wrong direction.
    #[arg(long, default_value_t = -1.0, allow_hyphen_values = true)]
    ball_track_gain: f64,

    /// How long (seconds) to keep tracking the ball after it disappears from view.
    /// During this window the last known world-frame position is projected back into
    /// the current trunk frame using odometry, so the head keeps pointing toward
    /// where the ball was even if the robot rotates or translates slightly.
    #[arg(long, default_value_t = 3.0)]
    ball_memory_secs: f64,

    /// Bounding-box inflation factor for ball size estimation.
    /// SSD detections typically pad the bbox 15-25% beyond the actual object edge,
    /// making the ball appear larger (= closer / higher) than it really is.
    /// A value of 1.2 means the bbox is treated as 20% bigger than the ball.
    #[arg(long, default_value_t = 1.2)]
    ball_bbox_scale: f64,

    /// Path to ball-kick policy ONNX file.
    /// When provided, the kick-ball policy is always active (replaces the walking
    /// policy). Ball detection is auto-enabled. Kick direction is always forward.
    #[arg(long)]
    ball_kick: Option<String>,

    /// Target kick speed in m/s fed to the kick-ball policy as the target ball
    /// velocity magnitude (always forward, i.e. [kick_speed, 0, 0] in body frame).
    #[arg(long, default_value_t = 1.0)]
    kick_speed: f64,

    /// Disable head tracking when ball detection is active.
    /// By default the head follows the detected ball; this flag turns that off.
    #[arg(long)]
    no_ball_track: bool,

}


/// Main robot runtime
struct Runtime {
    motor_controller: MotorController,
    imu_controller: AnyImuController,
    policy: Policy,
    controller: Controller,
    control_freq: u32,
    odo_freq: u32,
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
    ball_track_gain: f64,  // Proportional gain for ball-tracking head control
    ball_world_pos: Option<[f64; 3]>,  // Last known ball position in world frame (None = never seen)
    ball_last_seen: Option<Instant>,   // When the ball was last detected
    ball_memory_secs: f64,             // How long to remember ball position after losing sight of it
    ball_bbox_scale: f64,              // Bbox inflation factor for depth/Z estimation
    ball_kick_policy: Option<Policy>,  // Kick-ball policy (replaces walking when loaded)
    kick_speed: f64,                   // Target kick speed (m/s) fed to kick-ball policy
    ball_track: bool,                  // Whether to move the head toward the detected ball
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
    // Fold robot mode
    fold_mode: bool,
    fold_active: bool,      // true = phase cycling (folding/unfolding)
    fold_phase: f64,        // ∈ [0, 1), advances at dt/fold_period per step
    fold_period: f64,
    fold_action_scale: f64,
    // Standing policy
    roller_mode: bool,
    motorized_wheel_mode: bool,
    motorized_wheel_action_scale: f64,
    invert_wheel: bool,
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
    // Brain JPEG stream (port 9871): camera frames pushed to a connected client
    jpeg_listener: Option<std::net::TcpListener>,
    jpeg_client: Option<std::net::TcpStream>,
    // Brain command socket (port 9872): UDP, receives [vx,vy,wz] as 3×f32 LE
    cmd_socket: Option<UdpSocket>,
    // Ball-detection JSON stream (port 9873): newline-delimited JSON pushed to a client
    detection_listener: Option<std::net::TcpListener>,
    detection_client: Option<std::net::TcpStream>,
    // Background camera capture thread
    camera_stream: Option<camera::CameraStream>,
    // Latest velocity command received from the brain (applied when gamepad is idle)
    brain_command: [f64; 3],
    // Inline odometry engine (None if no URDF provided)
    odometry_engine: Option<odometry::Odometry>,
    /// Latest odometry estimate [x, y, z, yaw] in world frame (metres / radians)
    pub odometry: [f64; 4],
    // Last successfully read motor state (used as fallback on read failure)
    last_motor_state: motor::MotorState,
    // Cumulative count of motor read failures (shown in status line)
    read_error_count: u64,
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
        } else if args.bno055 {
            let use_projected_gravity = !args.raw_accelerometer;
            let ctrl = ImuController::new_default_with_mode(use_projected_gravity)
                .context("Failed to initialize IMU controller (BNO055 on /dev/i2c-1)")?;
            if use_projected_gravity {
                println!("✓ IMU controller initialized (BNO055) - using projected gravity from quaternion");
            } else {
                println!("✓ IMU controller initialized (BNO055) - using raw accelerometer");
            }
            AnyImuController::Bno055(ctrl)
        } else {
            // Default: BMI088 (also selected explicitly via --bmi088)
            let ctrl = Bmi088Controller::new_default()
                .context("Failed to initialize IMU controller (BMI088 on /dev/i2c-1)")?;
            println!("✓ IMU controller initialized (BMI088) - using projected gravity from Madgwick filter");
            AnyImuController::Bmi088(ctrl)
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
            println!("✓ Loading ONNX model from: {}", walking_path);
            let mut p = Policy::new_onnx(walking_path).context("Failed to load ONNX model")?;
            if let Some(ref standing_path) = args.standing {
                println!("✓ Loading standing ONNX model from: {}", standing_path);
                match p.add_standing(standing_path) {
                    Ok(()) => println!("✓ Policy switching enabled (threshold: 0.05 for cmd magnitude)"),
                    Err(e) => eprintln!("⚠  Failed to load standing policy (switching disabled): {}", e),
                }
            }
            p
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

        // In fold mode, load fold policy and disable standing policy switching
        if args.fold {
            policy.set_standing_disabled(true);
            println!("✓ Fold mode: standing policy switching disabled");
            if let Some(ref fold_path) = args.fold_policy {
                println!("✓ Loading fold policy from: {}", fold_path);
                match policy.add_fold(fold_path) {
                    Ok(()) => println!("  Fold period: {:.2}s (B button to toggle)", args.fold_period),
                    Err(e) => eprintln!("⚠  Failed to load fold policy (B button disabled): {}", e),
                }
            } else {
                eprintln!("⚠  --fold specified but no --fold-policy given; running without fold cycling");
            }
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
            match policy.add_ground_pick(gp_path) {
                Ok(()) => println!("  Ground pick period: {:.2}s (A button to trigger)", args.ground_pick_period),
                Err(e) => eprintln!("⚠  Failed to load ground pick policy (A button disabled): {}", e),
            }
        }

        // Load jump model if specified
        if let Some(ref jump_path) = args.jump {
            println!("✓ Loading jump ONNX model from: {}", jump_path);
            match policy.add_jump(jump_path) {
                Ok(()) => println!("  Jump period: {:.2}s (X button to trigger)", args.jump_period),
                Err(e) => eprintln!("⚠  Failed to load jump policy (X button disabled): {}", e),
            }
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
            odo_freq: {
                // Clamp to a multiple of policy freq; fall back to policy freq if invalid
                let of = args.odo_freq.max(args.freq);
                if of % args.freq == 0 { of } else { args.freq * (of / args.freq) }
            },
            pid_gains: (args.kp, args.ki, args.kd),
            pitch_offset: args.pitch_offset,
            last_action: [0.0; NUM_MOTORS],
            // In fold mode, initialize command to phase=0 (STAND target) so the fold
            // policy sees a valid in-distribution command before B is first pressed.
            command: if args.fold { [1.0, 0.0, 0.0] } else { [args.vel_x, args.vel_y, args.vel_z] },
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
                let mut p = if args.fold {
                    FOLD_DEFAULT_POSITION
                } else {
                    DEFAULT_POSITION
                };
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
                // neck_pitch_default / head_pitch_default override the table.
                // In fold mode FOLD_DEFAULT_POSITION already has the correct values
                // (neck=0.5236, head=-0.5236); skip the override so microduck defaults
                // (-0.5 / +0.5) don't clobber them.
                if !args.fold {
                    p[5] = args.neck_pitch_default;
                    p[6] = args.head_pitch_default;
                }
                p
            },
            head_max: args.head_max,
            head_alpha: args.head_alpha,
            ball_track_gain: args.ball_track_gain,
            ball_world_pos: None,
            ball_last_seen: None,
            ball_memory_secs: args.ball_memory_secs,
            ball_bbox_scale: args.ball_bbox_scale,
            ball_kick_policy: if let Some(ref path) = args.ball_kick {
                let expanded = if path.starts_with("~/") {
                    format!("{}/{}", std::env::var("HOME").unwrap_or_default(), &path[2..])
                } else {
                    path.clone()
                };
                println!("✓ Loading kick-ball policy from: {}", expanded);
                Some(Policy::new_onnx(&expanded).context("Failed to load kick-ball policy")?)
            } else {
                None
            },
            kick_speed: args.kick_speed,
            ball_track: !args.no_ball_track,
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
            fold_mode: args.fold,
            fold_active: false,
            fold_phase: 0.0,
            fold_period: args.fold_period,
            fold_action_scale: args.fold_action_scale,
            roller_mode: args.roller,
            motorized_wheel_mode: args.motorized_wheel,
            motorized_wheel_action_scale: args.motorized_wheel_action_scale,
            invert_wheel: args.invert_wheel,
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
            jpeg_listener: if args.stream {
                let addr = format!("0.0.0.0:{}", args.jpeg_port);
                let listener = std::net::TcpListener::bind(&addr)
                    .with_context(|| format!("Failed to bind JPEG listener on {}", addr))?;
                listener.set_nonblocking(true)
                    .context("Failed to set JPEG listener non-blocking")?;
                println!("JPEG stream listening on TCP port {}", args.jpeg_port);
                Some(listener)
            } else {
                None
            },
            jpeg_client: None,
            cmd_socket: if args.stream {
                let addr = format!("0.0.0.0:{}", args.cmd_port);
                let sock = UdpSocket::bind(&addr)
                    .with_context(|| format!("Failed to bind UDP command socket on {}", addr))?;
                sock.set_nonblocking(true)
                    .context("Failed to set command socket non-blocking")?;
                println!("UDP command socket listening on port {}", args.cmd_port);
                Some(sock)
            } else {
                None
            },
            detection_listener: if args.ball_detect || args.ball_kick.is_some() {
                let addr = format!("0.0.0.0:{}", args.ball_detect_port);
                let listener = std::net::TcpListener::bind(&addr)
                    .with_context(|| format!("Failed to bind detection listener on {}", addr))?;
                listener.set_nonblocking(true)
                    .context("Failed to set detection listener non-blocking")?;
                println!("Ball-detection stream listening on TCP port {}", args.ball_detect_port);
                Some(listener)
            } else {
                None
            },
            detection_client: None,
            camera_stream: if args.stream || args.ball_detect || args.ball_kick.is_some() {
                let need_detect = args.ball_detect || args.ball_kick.is_some();
                if need_detect {
                    println!("Starting camera + IMX500 ball detection ({}×{} @ {} fps) …",
                        args.cam_width, args.cam_height, args.cam_fps);
                } else {
                    println!("Starting camera capture ({}×{} @ {} fps) …",
                        args.cam_width, args.cam_height, args.cam_fps);
                }
                Some(camera::CameraStream::start(
                    args.cam_width, args.cam_height, args.cam_fps,
                    need_detect,
                ))
            } else {
                None
            },
            brain_command: [0.0; 3],
            odometry_engine: if args.stream || args.ball_detect || args.ball_kick.is_some() {
                println!("Odometry initialized (MJCF hardcoded chains)");
                Some(odometry::Odometry::new())
            } else {
                None
            },
            odometry: [0.0; 4],
            last_motor_state: motor::MotorState::default(),
            read_error_count: 0,
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
            println!("✓ Wheel motors set to velocity control (Operating Mode 1){}",
                     if self.invert_wheel { " [INVERTED]" } else { "" });
        }

        // Smoothly interpolate to default position over 1 second
        println!("Moving to default position over 1 second...");
        self.motor_controller.interpolate_to_position(&self.default_positions, Duration::from_secs(1))
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

    /// Lightweight inter-policy odometry update: read IMU only (no motor I/O),
    /// call odo.update() with the last-known joint angles.
    /// Used by the high-frequency odometry ticks between full control steps.
    fn imu_odo_step(&mut self) -> Result<()> {
        if self.odometry_engine.is_none() {
            return Ok(());
        }
        let imu_data = self.imu_controller.read()
            .context("Failed to read IMU (odo step)")?;
        // Extract angles before the mutable borrow of odometry_engine to avoid
        // the closure in from_fn capturing self while odo is live.
        let angles: [f64; 15] = std::array::from_fn(|i| self.last_motor_state.positions[i]);
        if let Some(odo) = &mut self.odometry_engine {
            odo.update(&angles, imu_data.quat);
            let p = odo.position;
            self.odometry = [p[0], p[1], p[2], odo.yaw];
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

            // Handle B button (East): fold toggle in fold mode, body pose mode otherwise
            let b_pressed = self.controller.is_button_pressed("East");
            if b_pressed && !self.b_button_prev_state {
                if self.fold_mode {
                    // Toggle fold cycling; fold policy always runs, phase cmd controls stand vs fold
                    self.fold_active = !self.fold_active;
                    if self.fold_active {
                        self.fold_phase = 0.0;
                        self.action_scale = self.fold_action_scale;
                        println!("▼ FOLD: cycling ON (period={:.1}s, action_scale={:.2})", self.fold_period, self.action_scale);
                    } else {
                        self.fold_phase = 0.0;
                        self.command = [1.0, 0.0, 0.0]; // phase=0 → STAND target
                        println!("▲ FOLD: cycling OFF → holding STAND");
                    }
                } else {
                    self.body_pose_mode = !self.body_pose_mode;
                    if self.body_pose_mode {
                        self.body_cmd = [0.0; 3];
                        println!("BODY POSE mode: ON (L-stick up/down: z ±25mm, R-stick up: pitch ±20°, R-stick down: roll ±20°)");
                    } else {
                        self.body_cmd = [0.0; 3];
                        println!("BODY POSE mode: OFF");
                    }
                }
            }
            self.b_button_prev_state = b_pressed;

            // Handle X button (North) to trigger one jump cycle (auto-exits on completion)
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
                let gamepad_target = [
                    left_x as f64 * self.max_linear_vel,
                    -left_y as f64 * self.max_linear_vel,
                    -right_y as f64 * 1.5 * self.max_angular_vel,
                ];
                // Brain command active when all gamepad sticks are at deadzone (== 0.0)
                let gamepad_idle = left_x == 0.0 && left_y == 0.0 && right_y == 0.0;
                let target_cmd = if gamepad_idle && !self.ground_pick_active && !self.jump_active {
                    self.brain_command
                } else {
                    gamepad_target
                };
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

        // Fold robot axis remap: physical IMU reads (x=up, y=left, z=back).
        // Remap to standard body frame (x=forward, y=left, z=up):
        //   accel/gyro:  new_x = -old_z,  new_y = old_y,  new_z = old_x
        //   quaternion:  post-multiply by +90° around Y = [√2/2, 0, √2/2, 0]
        //     q_std = q_sensor * [s, 0, s, 0]  where s = √2/2
        //     w_new = (qw-qy)*s,  x_new = (qx-qz)*s,  y_new = (qw+qy)*s,  z_new = (qx+qz)*s
        //   Verified: when upright q_sensor≈[s,0,-s,0] → q_std=[1,0,0,0].
        //   The quat fix is only needed for streaming (the policy does not use quat).
        if self.fold_mode {
            let [ax, ay, az] = imu_data.accel;
            let [gx, gy, gz] = imu_data.gyro;
            imu_data.accel = [-az, ay, ax];
            imu_data.gyro  = [-gz, gy, gx];
            let s = std::f64::consts::FRAC_1_SQRT_2;
            let [qw, qx, qy, qz] = imu_data.quat;
            imu_data.quat = [
                (qw - qy) * s,
                (qx - qz) * s,
                (qw + qy) * s,
                (qx + qz) * s,
            ];
        }

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

        // Fall detection: accel is projected gravity (unit vector, body frame).
        // After the fold-mode axis remap above, accel is always in standard frame: z+ = up.
        // Upright means accel[2] ≈ -1.0 for both standard and fold robots.
        // Debounce: only trigger after 0.2s of continuous detection.
        let upright_axis = 2usize;
        if self.battery_benchmark {
            // Benchmark fall detection: auto-recover via standing policy (command → 0)
            if self.benchmark_recovering {
                if imu_data.accel[upright_axis] < -0.5 {
                    // Robot is upright again: resume benchmark walking
                    self.benchmark_recovering = false;
                    self.fall_detected_since = None;
                    println!("▶ Benchmark: upright, resuming walk");
                }
            } else if imu_data.accel[upright_axis] > -0.5 {
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
            if imu_data.accel[upright_axis] > -0.5 {
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

        // Read motor state; on transient failure reuse the last valid reading
        let mut motor_state = match self.motor_controller.read_state() {
            Ok(state) => {
                self.last_motor_state = state.clone();
                state
            }
            Err(_) => {
                self.read_error_count += 1;
                self.last_motor_state.clone()
            }
        };

        // In motorized-wheel mode the bulk read fails for velocity-mode motors,
        // returning stale (or zero) wheel velocities. Patch with a dedicated
        // sync_read on just the two wheel IDs, which works correctly in Mode 1.
        if self.motorized_wheel_mode {
            match self.motor_controller.read_wheel_velocities() {
                Ok((lv, rv)) => {
                    motor_state.velocities[WHEEL_MOTOR_INDICES[0]] = lv;
                    motor_state.velocities[WHEEL_MOTOR_INDICES[1]] = rv;
                }
                Err(e) => eprintln!("Wheel vel read error: {e}"),
            }
        }

        // Digital twin streaming: send 39 × f32 LE per frame:
        //   [0..3]   IMU quaternion [w, x, y, z]
        //   [4..18]  joint positions (runtime motor order)
        //   [19..33] motor currents in mA (same order)
        //   [34..35] odometry x, y in metres (0.0 if odometry not running)
        //   [36..38] ball world position [x, y, z] in metres (NaN if not seen recently)
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
                // Ball world position: NaN if not detected recently
                let ball_valid = self.ball_world_pos.is_some()
                    && self.ball_last_seen
                        .map(|t| t.elapsed().as_secs_f64() < self.ball_memory_secs)
                        .unwrap_or(false);
                let ball = if ball_valid {
                    let b = self.ball_world_pos.unwrap();
                    [b[0] as f32, b[1] as f32, b[2] as f32]
                } else {
                    [f32::NAN, f32::NAN, f32::NAN]
                };
                let mut buf = [0u8; 39 * 4];
                let floats: [f32; 39] = [
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
                    // [34..35] odometry x, y in metres
                    self.odometry[0] as f32,
                    self.odometry[1] as f32,
                    // [36..38] ball world position [x, y, z] (NaN if not seen recently)
                    ball[0], ball[1], ball[2],
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

        // ── Brain JPEG stream (port 9871) ─────────────────────────────────────
        if let Some(ref listener) = self.jpeg_listener {
            if self.jpeg_client.is_none() {
                if let Ok((stream, addr)) = listener.accept() {
                    let _ = stream.set_nodelay(true);
                    println!("Brain JPEG client connected: {}", addr);
                    self.jpeg_client = Some(stream);
                }
            }
        }
        if self.jpeg_client.is_some() {
            if let Some(ref cam) = self.camera_stream {
                // Consume the latest frame (None = no new frame since last send)
                let frame = cam.latest_frame.lock().unwrap().take();
                if let Some(jpeg) = frame {
                    let len_bytes = (jpeg.len() as u32).to_le_bytes();
                    let ok = self.jpeg_client.as_mut().map(|c| {
                        c.write_all(&len_bytes).is_ok() && c.write_all(&jpeg).is_ok()
                    }).unwrap_or(false);
                    if !ok {
                        println!("Brain JPEG client disconnected");
                        self.jpeg_client = None;
                    }
                }
            }
        }

        // ── Ball-detection: accept client, consume latest JSON, stream + track ──
        if let Some(ref listener) = self.detection_listener {
            if self.detection_client.is_none() {
                if let Ok((stream, addr)) = listener.accept() {
                    let _ = stream.set_nodelay(true);
                    println!("Ball-detection client connected: {}", addr);
                    self.detection_client = Some(stream);
                }
            }
        }
        // Take the latest detection JSON once; reuse for both TCP and head tracking.
        let latest_det = self.camera_stream.as_ref()
            .and_then(|cam| cam.latest_detections.lock().unwrap().take());

        if let Some(ref json_line) = latest_det {
            // ── TCP stream to viewer ──────────────────────────────────────────
            if self.detection_client.is_some() {
                let mut payload = json_line.clone().into_bytes();
                payload.push(b'\n');
                let ok = self.detection_client.as_mut()
                    .map(|c| c.write_all(&payload).is_ok())
                    .unwrap_or(false);
                if !ok {
                    println!("Ball-detection client disconnected");
                    self.detection_client = None;
                }
            }
            // ── Ball world-frame update ───────────────────────────────────────
            // If the camera sees the ball, convert its trunk-frame position to
            // world frame via odometry and store it for persistence.
            if let Some(ball_trunk) = extract_ball_trunk_pos(json_line, self.ball_bbox_scale) {
                let ball_world = trunk_pos_to_world(ball_trunk, self.odometry, imu_data.quat);
                self.ball_world_pos = Some(ball_world);
                self.ball_last_seen = Some(Instant::now());
            }
        }

        // ── Head tracking (uses world-frame memory, corrected by odometry) ────
        // Runs as long as the ball was seen recently, unless --no-ball-track is set.
        if self.ball_track {
            if let Some(ball_world) = self.ball_world_pos {
                let age_secs = self.ball_last_seen
                    .map(|t| t.elapsed().as_secs_f64())
                    .unwrap_or(f64::MAX);
                if age_secs < self.ball_memory_secs {
                    let ball_trunk = world_pos_to_trunk(ball_world, self.odometry, imu_data.quat);
                    apply_head_tracking(
                        &mut self.head_offsets,
                        ball_trunk,
                        self.ball_track_gain,
                        self.head_alpha,
                    );
                }
            }
        }


        // ── Brain UDP command socket (port 9872) ──────────────────────────────
        // Drain all pending datagrams — only the last one matters.
        if let Some(ref sock) = self.cmd_socket {
            let mut buf = [0u8; 12]; // 3 × f32
            loop {
                match sock.recv(&mut buf) {
                    Ok(12) => {
                        let vx = f32::from_le_bytes(buf[0..4].try_into().unwrap()) as f64;
                        let vy = f32::from_le_bytes(buf[4..8].try_into().unwrap()) as f64;
                        let wz = f32::from_le_bytes(buf[8..12].try_into().unwrap()) as f64;
                        self.brain_command = [vx, vy, wz];
                    }
                    Ok(_) => {}  // wrong size, ignore
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
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

        // ── Build walking observation (used for logging and normal walking policy) ──
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

        // ── Kick-ball policy (overrides walking policy when loaded) ──────────────
        // Build a 54D kick-ball observation and run the dedicated ONNX model.
        // Ball position defaults to [0,0,0] (body frame) when not yet detected.
        // Kick target velocity is always [kick_speed, 0, 0] (straight forward).
        let action = if self.ball_kick_policy.is_some() {
            let ball_body = if let Some(bw) = self.ball_world_pos {
                world_pos_to_trunk(bw, self.odometry, imu_data.quat)
            } else {
                [0.0_f64; 3]
            };
            let kick_vel = [self.kick_speed, 0.0_f64, 0.0_f64];
            let kick_obs = Observation::new_kick_ball(
                &imu_data,
                &motor_state,
                &self.last_action,
                &self.default_positions,
                ball_body,
                kick_vel,
            );
            if self.policy_enabled {
                self.ball_kick_policy.as_mut().unwrap()
                    .infer(&kick_obs, &[0.0; 3], false)
                    .context("Failed to run kick-ball policy inference")?
            } else {
                [0.0f32; NUM_MOTORS]
            }
        } else {
            // ── Normal walking/standing policy ────────────────────────────────
            if self.policy_enabled {
                self.policy.infer(&observation, effective_command, self.mouth_enabled)
                    .context("Failed to run policy inference")?
            } else {
                [0.0f32; NUM_MOTORS]
            }
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
        // For motorized-wheel mode, remap the policy output to the correct motor indices.
        //
        // The policy outputs 14 values in this order:
        //   [0..11] = 12 position joints: left_hip_yaw/roll/pitch/knee, neck_pitch, head_pitch/yaw/roll,
        //             right_hip_yaw/roll/pitch/knee  (ctrl order skipping wheels at ctrl[4] and ctrl[13])
        //   [12]    = left_wheel velocity  (ctrl[4])
        //   [13]    = right_wheel velocity (ctrl[13] in sim = motor index 14 in runtime)
        //
        // policy.rs legacy mapping inserts mouth=0 at index 9:
        //   action[k] = policy[k] for k < 9, action[k+1] = policy[k] for k >= 9.
        //
        // So: action[4]=policy[4]=neck_pitch (not left_wheel), action[13]=policy[12]=left_wheel_vel.
        // We must remap before computing motor_targets. self.last_action stays as the legacy-mapped
        // action so that new_motorized_wheel() obs (which skips index 9) gives back the original 14D.
        let action_for_targets = if self.motorized_wheel_mode {
            let mut mw = [0.0f32; NUM_MOTORS];
            mw[0]  = action[0];   // left_hip_yaw   (policy[0])
            mw[1]  = action[1];   // left_hip_roll   (policy[1])
            mw[2]  = action[2];   // left_hip_pitch  (policy[2])
            mw[3]  = action[3];   // left_knee       (policy[3])
            mw[4]  = action[13];  // left_wheel vel  (policy[12] → legacy action[13])
            mw[5]  = action[4];   // neck_pitch      (policy[4])
            mw[6]  = action[5];   // head_pitch      (policy[5])
            mw[7]  = action[6];   // head_yaw        (policy[6])
            mw[8]  = action[7];   // head_roll       (policy[7])
            mw[9]  = 0.0;         // mouth (controlled externally)
            mw[10] = action[8];   // right_hip_yaw   (policy[8]; k<9 so action[8])
            mw[11] = action[10];  // right_hip_roll  (policy[9]  → legacy action[10])
            mw[12] = action[11];  // right_hip_pitch (policy[10] → legacy action[11])
            mw[13] = action[12];  // right_knee      (policy[11] → legacy action[12])
            mw[14] = action[14];  // right_wheel vel (policy[13] → legacy action[14])
            mw
        } else {
            action
        };

        let mut motor_targets = [0.0f64; NUM_MOTORS];
        for i in 0..NUM_MOTORS {
            motor_targets[i] = self.default_positions[i] + effective_action_scale * action_for_targets[i] as f64;
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
            // action_for_targets[WHEEL_MOTOR_INDICES] holds the correct wheel velocities after remap.
            let sign = if self.invert_wheel { -1.0 } else { 1.0 };
            let left_vel  = sign * action_for_targets[WHEEL_MOTOR_INDICES[0]] as f64 * WHEEL_MAX_VEL * self.motorized_wheel_action_scale;
            let right_vel = sign * action_for_targets[WHEEL_MOTOR_INDICES[1]] as f64 * WHEEL_MAX_VEL * self.motorized_wheel_action_scale;
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

        // Advance jump phase; one-shot (clamp at 1.0), auto-exit on completion
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

        // Advance fold phase continuously while fold_active; hold [1,0,0] when idle
        if self.fold_active {
            let dt = 1.0 / self.control_freq as f64;
            self.fold_phase = (self.fold_phase + dt / self.fold_period) % 1.0;
            let two_pi_phase = 2.0 * std::f64::consts::PI * self.fold_phase;
            self.command = [two_pi_phase.cos(), two_pi_phase.sin(), 0.0];
        }

        Ok(())
    }

    /// Run the main control loop
    async fn run(&mut self, shutdown_flag: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Result<()> {
        println!("Starting control loop at {} Hz (policy), {} Hz (odometry)", self.control_freq, self.odo_freq);

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

        // policy_every: run full control_step() once every N odo ticks.
        // odo_freq >= control_freq and odo_freq is a multiple of control_freq.
        let policy_every = self.odo_freq / self.control_freq;
        let dt = Duration::from_secs_f64(1.0 / self.odo_freq as f64);
        let mut iteration: u64 = 0;   // policy-step counter
        let mut odo_tick: u64 = 0;    // every-tick counter
        let mut total_time = Duration::ZERO;
        let mut min_loop_time = Duration::from_secs(999999);
        let mut max_loop_time = Duration::ZERO;
        let mut last_report_iteration: u64 = 0;
        let mut missed_deadlines: u64 = 0;

        if policy_every > 1 {
            println!("Odometry running at {} Hz (policy at {} Hz)", self.odo_freq, self.control_freq);
        }

        // Create tokio interval with MissedTickBehavior::Burst
        // Burst mode will catch up on missed ticks rather than skipping them
        let mut interval = tokio::time::interval(dt);
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Burst);

        while !shutdown_flag.load(std::sync::atomic::Ordering::SeqCst) {
            // Wait for next tick
            interval.tick().await;
            odo_tick += 1;

            let is_policy_tick = odo_tick % policy_every as u64 == 0;

            if !is_policy_tick {
                // IMU + odometry only — no motor I/O, no policy inference
                if let Err(e) = self.imu_odo_step() {
                    eprintln!("Error in odo step: {}", e);
                }
                continue;
            }

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

                let err_str = if self.read_error_count > 0 {
                    format!(" | ReadErr: {}", self.read_error_count)
                } else {
                    String::new()
                };

                println!(
                    "⏱️  Runtime: {:.1}s | Iter: {} | Avg: {:.2}ms ({:.1}%) | Min: {:.2}ms | Max: {:.2}ms | Jitter: {:.2}ms | Freq: {:.1}Hz | {}{}{}",
                    total_elapsed,
                    iteration,
                    avg_time.as_secs_f64() * 1000.0,
                    (avg_time.as_secs_f64() / dt.as_secs_f64()) * 100.0,
                    min_loop_time.as_secs_f64() * 1000.0,
                    max_loop_time.as_secs_f64() * 1000.0,
                    jitter * 1000.0,
                    actual_freq,
                    current_str,
                    cmd_str,
                    err_str
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

// ── Ball tracking (FK-based, torso camera) ────────────────────────────────────
//
// The torso camera is fixed to trunk_base — moving the head doesn't change what
// it sees, so image-based visual servoing doesn't close the loop.  Instead:
//
//  1. Estimate ball 3D position in trunk_base frame using the pinhole camera
//     model + known ball diameter (60 mm).
//  2. Compute the azimuth and elevation of the ball from the neck joint.
//  3. Set head_yaw / head_pitch offsets to point the head at that position.
//
// Torso camera (from robot_walk.xml):
//   pos  = (0.0506, 0, 0.0305) in trunk_base
//   quat = (0.5, 0.5, -0.5, -0.5)  →  looks in trunk +X, image-right = trunk -Y,
//                                       image-up = trunk +Z
//
// Signs for head_yaw / head_pitch depend on motor zero conventions.
// Use --ball-track-gain with a negative value to invert if head moves wrong way.
/// Parse detection JSON and return the ball's 3-D position [x, y, z] in the
/// trunk_base frame, or `None` if no sports-ball / apple is present.
///
/// Pi AI Camera (IMX500) at 640×480: estimated focal length ≈ 280 px
/// (derived from 2.75 mm lens / 6.287 mm sensor width / 2028 px full-res, scaled to 640 px).
///
/// Torso camera in trunk_base (from robot_walk.xml):
///   pos  = (0.0506, 0.0, 0.0305)
///   quat = (0.5, 0.5, -0.5, -0.5) → R = [[0,0,-1],[-1,0,0],[0,1,0]]
///   image-right = trunk -Y, image-up = trunk +Z, optical axis = trunk +X
fn extract_ball_trunk_pos(json: &str, bbox_scale: f64) -> Option<[f64; 3]> {
    const TARGET_CLASSES: [&str; 2] = ["sports ball", "apple"];
    const FX: f64 = 280.0;
    const FY: f64 = 280.0;
    const CX_IMG: f64 = 320.0;
    const CY_IMG: f64 = 240.0;
    const BALL_DIAMETER: f64 = 0.065; // 65 mm

    let mut best_score = 0.0_f64;
    let mut best_box: Option<[f64; 4]> = None;

    for chunk in json.split('{').skip(1) {
        let score = chunk.split("\"score\":").nth(1)
            .and_then(|s| s.split([',', '}']).next())
            .and_then(|s| s.trim().parse::<f64>().ok())
            .unwrap_or(0.0);
        if score <= best_score { continue; }
        let is_target = TARGET_CLASSES.iter().any(|&cls| {
            chunk.contains(&format!("\"class\":\"{cls}\""))
        });
        if !is_target { continue; }
        if let Some(arr) = chunk.split("\"box\":[").nth(1).and_then(|s| s.split(']').next()) {
            let nums: Vec<f64> = arr.split(',')
                .filter_map(|v| v.trim().parse().ok())
                .collect();
            if nums.len() == 4 {
                best_score = score;
                best_box = Some([nums[0], nums[1], nums[2], nums[3]]);
            }
        }
    }

    let [bx, by, bw, bh] = best_box?;
    let cx = bx + bw / 2.0;
    let cy = by + bh / 2.0;
    // Divide by bbox_scale to correct for SSD padding: bbox is typically ~20% larger
    // than the actual ball, making it appear bigger (closer/higher) than it is.
    let apparent_px = (bw.max(bh) / bbox_scale).max(1.0);
    let z_depth = FX * BALL_DIAMETER / apparent_px; // depth along camera optical axis

    // The camera is physically mounted rotated 90° CW; view_camera.py rotates 90° CCW
    // to correct the display.  Detection bounding boxes are in RAW sensor coordinates,
    // so the axes are swapped relative to a normally-mounted camera:
    //   raw cx (horizontal in raw) → physical UP/DOWN  → trunk Z
    //   raw cy (vertical in raw)   → physical LEFT/RIGHT → trunk Y
    //
    // After the 90° CCW display rotation: display_x = raw cy, display_y = 639 - raw cx
    //   → trunk_y driven by (cy - CY_IMG):  cy > 240 = display right = trunk -Y (robot right)
    //   → trunk_z driven by (cx - CX_IMG):  cx > 320 = display up    = trunk +Z (up)
    Some([
        0.0506 + z_depth,
        0.0    - (cy - CY_IMG) / FY * z_depth,
        0.0305 + (cx - CX_IMG) / FX * z_depth,
    ])
}

/// Rotate vector `v` by unit quaternion `q = [w, x, y, z]` using Rodrigues formula.
#[inline]
fn quat_rotate(v: [f64; 3], q: [f64; 4]) -> [f64; 3] {
    let [qw, qx, qy, qz] = q;
    let [vx, vy, vz] = v;
    // t = 2 * (q_vec × v)
    let tx = 2.0 * (qy * vz - qz * vy);
    let ty = 2.0 * (qz * vx - qx * vz);
    let tz = 2.0 * (qx * vy - qy * vx);
    // v' = v + qw * t + q_vec × t
    [
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    ]
}

/// Convert a point in trunk_base frame to the world (odometry) frame.
///
/// Uses the full IMU quaternion so trunk pitch/roll are correctly accounted for.
/// `odo` is `[x, y, z, yaw]` — only `[0..3]` (position) is used here.
#[inline]
fn trunk_pos_to_world(p: [f64; 3], odo: [f64; 4], imu_quat: [f64; 4]) -> [f64; 3] {
    let r = quat_rotate(p, imu_quat);
    [odo[0] + r[0], odo[1] + r[1], odo[2] + r[2]]
}

/// Convert a point in the world (odometry) frame back to trunk_base frame.
///
/// Inverse of `trunk_pos_to_world` — rotates by conjugate quaternion.
#[inline]
fn world_pos_to_trunk(p: [f64; 3], odo: [f64; 4], imu_quat: [f64; 4]) -> [f64; 3] {
    let dx = p[0] - odo[0];
    let dy = p[1] - odo[1];
    let dz = p[2] - odo[2];
    // conjugate: negate xyz part of the quaternion
    let q_inv = [imu_quat[0], -imu_quat[1], -imu_quat[2], -imu_quat[3]];
    quat_rotate([dx, dy, dz], q_inv)
}

/// Update head_offsets to point toward `ball_trunk` (a point in trunk_base frame).
///
/// Uses the neck joint origin as the pivot, computes azimuth (→ head_yaw) and
/// elevation (→ head_pitch), and applies them via an EMA with factor `alpha`.
fn apply_head_tracking(head_offsets: &mut [f64; 4], ball_trunk: [f64; 3], gain: f64, alpha: f64) {
    // Neck joint origin in trunk_base (from robot_walk.xml long_neck_plate2 body pos)
    const NECK: [f64; 3] = [0.0375, 0.0146, 0.0552];

    let dx = ball_trunk[0] - NECK[0];
    let dy = ball_trunk[1] - NECK[1];
    let dz = ball_trunk[2] - NECK[2];
    let horiz = (dx * dx + dy * dy).sqrt().max(1e-3);

    // Azimuth in trunk XY plane: positive = ball to left (+Y), negative = right (-Y)
    let azimuth   = dy.atan2(dx);
    // Elevation: positive = above, negative = below
    let elevation = dz.atan2(horiz);

    // head_yaw (index 2) tracks azimuth; head_pitch (index 1) tracks elevation.
    // Both use the same gain sign (default -1.0 works for both axes).
    let yaw_target   = gain * azimuth;
    let pitch_target = gain * elevation;

    head_offsets[2] += alpha * (yaw_target   - head_offsets[2]);
    head_offsets[1] += alpha * (pitch_target - head_offsets[1]);
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
