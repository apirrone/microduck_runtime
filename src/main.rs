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
use std::time::{Duration, Instant};

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

    /// Position P gain for motors
    #[arg(long, default_value_t = 400)]
    kp: u16,

    /// Position I gain for motors
    #[arg(long, default_value_t = 0)]
    ki: u16,

    /// Position D gain for motors
    #[arg(long, default_value_t = 0)]
    kd: u16,
}


/// Main robot runtime
struct Runtime {
    motor_controller: MotorController,
    imu_controller: ImuController,
    policy: Policy,
    control_freq: u32,
    pid_gains: (u16, u16, u16), // (kP, kI, kD)
    last_action: [f32; NUM_MOTORS],
    command: [f64; 3],
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
        let imu_controller = ImuController::new_default()
            .context("Failed to initialize IMU controller (BNO055 on /dev/i2c-1)")?;
        println!("✓ IMU controller initialized (BNO055)");

        // Initialize policy based on arguments
        let policy = if args.dummy {
            println!("✓ Using dummy policy (always outputs zeros)");
            Policy::new_dummy().context("Failed to create dummy policy")?
        } else if let Some(ref path) = args.model {
            println!("✓ Loading ONNX model from: {}", path);
            Policy::new_onnx(path).context("Failed to load ONNX model")?
        } else {
            println!("! No policy specified, using dummy policy");
            Policy::new_dummy().context("Failed to create dummy policy")?
        };

        Ok(Self {
            motor_controller,
            imu_controller,
            policy,
            control_freq: args.freq,
            pid_gains: (args.kp, args.ki, args.kd),
            last_action: [0.0; NUM_MOTORS],
            command: [0.0; 3],
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
        let imu_data = self.imu_controller.read()
            .context("Failed to read IMU data")?;

        // Read motor state
        let motor_state = self.motor_controller.read_state()
            .context("Failed to read motor state")?;

        // Build observation
        let observation = Observation::new(
            &imu_data,
            &self.command,
            &motor_state,
            &self.last_action,
        );

        // Run policy inference
        let action = self.policy.infer(&observation)
            .context("Failed to run policy inference")?;

        // Convert action offsets to motor targets: init_pos + action
        let mut motor_targets = [0.0f64; NUM_MOTORS];
        for i in 0..NUM_MOTORS {
            motor_targets[i] = DEFAULT_POSITION[i] + action[i] as f64;
        }

        self.motor_controller.write_goal_positions(&motor_targets)
            .context("Failed to write motor positions")?;

        // Update last action
        self.last_action = action;

        Ok(())
    }

    /// Run the main control loop
    fn run(&mut self, shutdown_flag: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Result<()> {
        println!("Starting control loop at {} Hz", self.control_freq);

        let dt = Duration::from_secs_f64(1.0 / self.control_freq as f64);
        let mut iteration: u64 = 0;
        let mut total_time = Duration::ZERO;

        while !shutdown_flag.load(std::sync::atomic::Ordering::SeqCst) {
            let loop_start = Instant::now();

            // Run control step
            if let Err(e) = self.control_step() {
                eprintln!("Error in control step: {}", e);
                // Continue running despite errors
            }

            // Calculate timing
            let elapsed = loop_start.elapsed();
            total_time += elapsed;
            iteration += 1;

            // Print statistics every second
            if iteration % self.control_freq as u64 == 0 {
                let avg_time = total_time / iteration as u32;

                // Read leg currents
                let currents_result = self.motor_controller.read_leg_currents();
                let current_str = match currents_result {
                    Ok((left, right)) => format!("Left: {:.0} mA, Right: {:.0} mA, Total: {:.0} mA",
                                                left, right, left + right),
                    Err(_) => "Current read failed".to_string(),
                };

                println!(
                    "Running: {} iterations, avg loop time: {:.2} ms ({:.1}% of target) | Leg currents: {}",
                    iteration,
                    avg_time.as_secs_f64() * 1000.0,
                    (avg_time.as_secs_f64() / dt.as_secs_f64()) * 100.0,
                    current_str
                );
            }

            // Sleep to maintain desired frequency
            if elapsed < dt {
                std::thread::sleep(dt - elapsed);
            } else {
                eprintln!(
                    "Warning: Control loop overrun! Target: {:.2} ms, Actual: {:.2} ms",
                    dt.as_secs_f64() * 1000.0,
                    elapsed.as_secs_f64() * 1000.0
                );
            }
        }

        Ok(())
    }

    /// Shutdown the runtime safely
    fn shutdown(&mut self) -> Result<()> {
        println!("Shutting down runtime...");

        // Note: We do NOT disable motor torque on shutdown
        // Motors will maintain their last commanded position
        // To manually disable torque, use a separate command or power off the motors

        println!("✓ Runtime shutdown complete (motors remain enabled)");
        Ok(())
    }
}

fn main() -> Result<()> {
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

    // Run main loop (will exit when Ctrl+C is pressed)
    runtime.run(shutdown_flag)?;

    // Clean shutdown
    runtime.shutdown()?;

    Ok(())
}
