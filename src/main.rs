mod imu;
mod motor;
mod observation;
mod policy;

use anyhow::{Context, Result};
use clap::Parser;
use imu::ImuController;
use motor::{MotorController, NUM_MOTORS, DEFAULT_POSITION};
use observation::Observation;
use policy::Policy;
use serde::Serialize;
use std::fs::File;
use std::io::Write as IoWrite;
use std::time::{Duration, Instant};

/// Timestamped observation for recording
#[derive(Serialize, Debug, Clone)]
struct TimestampedObservation {
    timestamp: f64,  // Seconds since start
    observation: Vec<f32>,
}

/// Microduck Robot Runtime
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port for motor communication
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
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
    #[arg(short, long)]
    model: Option<String>,

    /// Path to standing policy ONNX model file (optional)
    #[arg(short, long)]
    standing: Option<String>,

    /// Position P gain for motors
    #[arg(long, default_value_t = 400)]
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

    /// Optional CSV log file to save observations and actions
    #[arg(long)]
    log_file: Option<String>,

    /// Enable imitation mode (adds phase observation)
    #[arg(long)]
    imitation: bool,

    /// Gait period in seconds (only used if --imitation is set)
    #[arg(long, default_value_t = 0.72)]
    gait_period: f64,

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
    #[arg(long, default_value_t = 1.0, allow_hyphen_values = true)]
    action_scale: f64,

    /// Enable recording mode: save observations to pickle file on Ctrl+C
    #[arg(short, long)]
    record: Option<String>,
}


/// Main robot runtime
struct Runtime {
    motor_controller: MotorController,
    imu_controller: ImuController,
    policy: Policy,
    control_freq: u32,
    pid_gains: (u16, u16, u16), // (kP, kI, kD)
    pitch_offset: f64,
    last_action: [f32; NUM_MOTORS],
    command: [f64; 3],
    action_scale: f64,
    log_file: Option<File>,
    start_time: Option<Instant>,
    step_counter: u64,
    use_imitation: bool,
    gait_period: f64,
    imitation_phase: f64,
    record_file: Option<String>,
    recorded_observations: Vec<TimestampedObservation>,
    policy_enabled: bool,  // Controls whether policy inference runs
}

impl Runtime {
    /// Create a new runtime instance
    fn new(args: &Args) -> Result<Self> {
        println!("Initializing microduck runtime...");

        // Initialize motor controller
        let motor_controller = MotorController::new(&args.port, args.baudrate)
            .context("Failed to initialize motor controller")?;
        println!("✓ Motor controller initialized on {} at {} baud", args.port, args.baudrate);

        // Initialize IMU controller (BNO055 on I2C)
        let mut imu_controller = ImuController::new_default()
            .context("Failed to initialize IMU controller (BNO055 on /dev/i2c-1)")?;
        println!("✓ IMU controller initialized (BNO055)");

        // Set gravity offset if specified
        if args.gravity_offset_x != 0.0 || args.gravity_offset_y != 0.0 || args.gravity_offset_z != 0.0 {
            imu_controller.set_gravity_offset([
                args.gravity_offset_x,
                args.gravity_offset_y,
                args.gravity_offset_z,
            ]);
        }

        // Initialize policy based on arguments
        let policy = if args.dummy {
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

        if args.pitch_offset != 0.0 {
            println!("⚠  Using pitch offset: {:.4} rad ({:.2}°)", args.pitch_offset, args.pitch_offset.to_degrees());
        }

        // Imitation mode configuration
        let gait_period = if args.imitation {
            // Priority 1: Use ONNX metadata if available
            // Priority 2: Fall back to command-line argument
            let period = policy.gait_period().unwrap_or(args.gait_period);

            if let Some(onnx_period) = policy.gait_period() {
                println!("✓ Imitation mode enabled");
                println!("  Using gait period from ONNX metadata: {:.4}s", onnx_period);
            } else {
                println!("✓ Imitation mode enabled");
                println!("  Using gait period from command-line: {:.4}s", args.gait_period);
            }

            period
        } else {
            args.gait_period  // Not used, but keep default
        };

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

        // Initialize log file if requested
        let mut log_file = None;
        if let Some(ref path) = args.log_file {
            let mut file = File::create(path)
                .context(format!("Failed to create log file: {}", path))?;

            // Write CSV header: step, time, obs_0..obs_N, action_0..action_13
            let obs_size = if args.imitation { 52 } else { 51 };
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

        // In recording mode, policy starts disabled and will be enabled after standby
        let policy_enabled = args.record.is_none();

        Ok(Self {
            motor_controller,
            imu_controller,
            policy,
            control_freq: args.freq,
            pid_gains: (args.kp, args.ki, args.kd),
            pitch_offset: args.pitch_offset,
            last_action: [0.0; NUM_MOTORS],
            command: [args.vel_x, args.vel_y, args.vel_z],
            action_scale: args.action_scale,
            log_file,
            start_time: None,
            step_counter: 0,
            use_imitation: args.imitation,
            gait_period,  // Use computed gait_period (from ONNX or CLI)
            imitation_phase: 0.0,
            record_file: args.record.clone(),
            recorded_observations: Vec::new(),
            policy_enabled,
        })
    }

    /// Initialize motors (enable torque, set PID gains, set to initial positions)
    fn initialize_motors(&mut self) -> Result<()> {
        println!("Initializing motors...");

        // Set PID gains
        let (kp, ki, kd) = self.pid_gains;
        println!("Setting PID gains: kP={}, kI={}, kD={}", kp, ki, kd);
        self.motor_controller.set_pid_gains(kp, ki, kd)
            .context("Failed to set PID gains")?;

        // Enable torque on all motors
        self.motor_controller.set_torque_enable(true)
            .context("Failed to enable motor torque")?;

        println!("✓ Motors initialized with PID gains and torque enabled");
        Ok(())
    }

    /// Run one control loop iteration
    fn control_step(&mut self) -> Result<()> {
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

        // Read motor state
        let motor_state = self.motor_controller.read_state()
            .context("Failed to read motor state")?;

        // Build observation with optional phase
        let phase = if self.use_imitation {
            Some(self.imitation_phase)
        } else {
            None
        };

        let observation = Observation::new(
            &imu_data,
            &self.command,
            &motor_state,
            &self.last_action,
            phase,
        );

        // Run policy inference (or hold default position if policy disabled)
        let action = if self.policy_enabled {
            self.policy.infer(&observation, &self.command)
                .context("Failed to run policy inference")?
        } else {
            // During standby, use zero actions (hold default position)
            [0.0f32; NUM_MOTORS]
        };

        // Convert action offsets to motor targets: init_pos + action_scale * action
        let mut motor_targets = [0.0f64; NUM_MOTORS];
        for i in 0..NUM_MOTORS {
            motor_targets[i] = DEFAULT_POSITION[i] + self.action_scale * action[i] as f64;
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

        self.motor_controller.write_goal_positions(&motor_targets)
            .context("Failed to write motor positions")?;

        // Update last action
        self.last_action = action;

        // Update phase for next step
        if self.use_imitation {
            let dt = 1.0 / self.control_freq as f64;
            self.imitation_phase += dt / self.gait_period;
            self.imitation_phase = self.imitation_phase % 1.0;  // Keep in [0, 1]
        }

        Ok(())
    }

    /// Run the main control loop
    async fn run(&mut self, shutdown_flag: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Result<()> {
        println!("Starting control loop at {} Hz (using tokio interval)", self.control_freq);

        // Set start time for CSV logging and recording
        self.start_time = Some(Instant::now());
        let runtime_start = Instant::now();

        // If recording mode and policy is disabled, schedule it to enable after 1 second
        let policy_enable_time = if !self.policy_enabled {
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

                // Read leg currents
                let currents_result = self.motor_controller.read_leg_currents();
                let current_str = match currents_result {
                    Ok((left, right)) => format!("Left: {:.0} mA, Right: {:.0} mA, Total: {:.0} mA",
                                                left, right, left + right),
                    Err(_) => "Current read failed".to_string(),
                };

                println!(
                    "⏱️  Runtime: {:.1}s | Iter: {} | Avg: {:.2}ms ({:.1}%) | Min: {:.2}ms | Max: {:.2}ms | Jitter: {:.2}ms | Freq: {:.1}Hz | {}",
                    total_elapsed,
                    iteration,
                    avg_time.as_secs_f64() * 1000.0,
                    (avg_time.as_secs_f64() / dt.as_secs_f64()) * 100.0,
                    min_loop_time.as_secs_f64() * 1000.0,
                    max_loop_time.as_secs_f64() * 1000.0,
                    jitter * 1000.0,
                    actual_freq,
                    current_str
                );

                // Reset jitter tracking for next interval
                min_loop_time = Duration::from_secs(999999);
                max_loop_time = Duration::ZERO;
                last_report_iteration = iteration;

                // Flush log file every second
                if let Some(ref mut file) = self.log_file {
                    file.flush().ok();
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

    /// Shutdown the runtime safely
    fn shutdown(&mut self) -> Result<()> {
        println!("Shutting down runtime...");

        // Save recorded observations if recording mode is enabled
        if let Some(ref path) = self.record_file {
            println!("Saving {} recorded observations to {}...",
                     self.recorded_observations.len(), path);

            let mut file = File::create(path)
                .context(format!("Failed to create recording file: {}", path))?;

            serde_pickle::to_writer(&mut file, &self.recorded_observations, Default::default())
                .context("Failed to serialize observations to pickle format")?;

            println!("✓ Recorded observations saved to {}", path);
        }

        // Flush and close log file if it exists
        if let Some(ref mut file) = self.log_file {
            file.flush()?;
            println!("✓ Log file flushed and closed");
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
    let args = Args::parse();

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

    // Initialize motors
    runtime.initialize_motors()?;

    // If recording mode is enabled, wait 1 second before starting the control loop
    // This gives the robot time to settle, then we'll record 1 second of standby
    if args.record.is_some() {
        println!("Recording mode: waiting 1 second before starting control loop...");
        tokio::time::sleep(Duration::from_secs(1)).await;
        println!("✓ Starting control loop (1s standby recording, then policy inference)");
    }

    // Run main loop (will exit when Ctrl+C is pressed)
    runtime.run(shutdown_flag).await?;

    // Clean shutdown
    runtime.shutdown()?;

    Ok(())
}
