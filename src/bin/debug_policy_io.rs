use anyhow::Result;
use clap::Parser;
use microduck_runtime::imu::ImuController;
use microduck_runtime::motor::{MotorController, NUM_MOTORS, DEFAULT_POSITION};
use microduck_runtime::observation::Observation;
use microduck_runtime::policy::Policy;
use std::fs::File;
use std::io::Write as IoWrite;
use std::thread;
use std::time::{Duration, Instant};

/// Policy Input/Output Debug Tool
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port for motor communication
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Motor communication baudrate
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Path to ONNX model file
    #[arg(short, long)]
    model: String,

    /// Pitch offset in radians
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    pitch_offset: f64,

    /// Duration to run in seconds
    #[arg(short, long, default_value_t = 5.0)]
    duration: f64,

    /// Output log file
    #[arg(short, long, default_value = "policy_debug.csv")]
    output: String,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("Policy I/O Debug Tool");
    println!("===================");
    println!();
    println!("This tool logs policy inputs and outputs for debugging");
    println!("Output file: {}", args.output);
    println!("Duration: {:.1}s", args.duration);
    println!();

    // Initialize components
    println!("Initializing motor controller...");
    let mut motor_controller = MotorController::new(&args.port, args.baudrate)?;
    println!("✓ Motor controller initialized");

    println!("Initializing IMU...");
    let mut imu = ImuController::new_default()?;
    println!("✓ IMU initialized");

    println!("Loading policy model...");
    let mut policy = Policy::new_onnx(&args.model)?;
    println!("✓ Policy loaded");
    println!();

    // Create log file
    let mut log_file = File::create(&args.output)?;

    // Write CSV header
    writeln!(log_file, "time,gyro_x,gyro_y,gyro_z,proj_grav_x,proj_grav_y,proj_grav_z,\
                        cmd_vx,cmd_vy,cmd_wz,\
                        joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,\
                        joint_pos_5,joint_pos_6,joint_pos_7,joint_pos_8,joint_pos_9,\
                        joint_pos_10,joint_pos_11,joint_pos_12,joint_pos_13,\
                        joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,\
                        joint_vel_5,joint_vel_6,joint_vel_7,joint_vel_8,joint_vel_9,\
                        joint_vel_10,joint_vel_11,joint_vel_12,joint_vel_13,\
                        action_0,action_1,action_2,action_3,action_4,action_5,action_6,\
                        action_7,action_8,action_9,action_10,action_11,action_12,action_13,\
                        target_0,target_1,target_2,target_3,target_4,target_5,target_6,\
                        target_7,target_8,target_9,target_10,target_11,target_12,target_13")?;

    println!("Press Enter to start logging...");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input)?;

    println!("Logging for {:.1}s...", args.duration);
    let start_time = Instant::now();
    let command = [0.0, 0.0, 0.0]; // Zero command
    let mut last_action = [0.0f32; NUM_MOTORS];

    let mut sample_count = 0;
    while start_time.elapsed().as_secs_f64() < args.duration {
        let iter_start = Instant::now();

        // Read sensors
        let mut imu_data = imu.read()?;

        // Apply pitch offset
        if args.pitch_offset != 0.0 {
            let cos_pitch = args.pitch_offset.cos();
            let sin_pitch = args.pitch_offset.sin();
            let x = imu_data.accel[0];
            let z = imu_data.accel[2];
            imu_data.accel[0] = x * cos_pitch + z * sin_pitch;
            imu_data.accel[2] = -x * sin_pitch + z * cos_pitch;
            let mag = (imu_data.accel[0].powi(2) + imu_data.accel[1].powi(2) + imu_data.accel[2].powi(2)).sqrt();
            if mag > 0.01 {
                imu_data.accel[0] /= mag;
                imu_data.accel[1] /= mag;
                imu_data.accel[2] /= mag;
            }
        }

        let motor_state = motor_controller.read_state()?;

        // Build observation
        let observation = Observation::new(&imu_data, &command, &motor_state, &last_action);

        // Run policy
        let action = policy.infer(&observation)?;

        // Compute targets
        let mut targets = [0.0; NUM_MOTORS];
        for i in 0..NUM_MOTORS {
            targets[i] = DEFAULT_POSITION[i] + action[i] as f64;
        }

        // Log data
        let t = start_time.elapsed().as_secs_f64();
        write!(log_file, "{:.4}", t)?;

        // Gyro
        for i in 0..3 {
            write!(log_file, ",{:.6}", imu_data.gyro[i])?;
        }

        // Projected gravity
        for i in 0..3 {
            write!(log_file, ",{:.6}", imu_data.accel[i])?;
        }

        // Command
        for i in 0..3 {
            write!(log_file, ",{:.6}", command[i])?;
        }

        // Joint positions (relative to default)
        for i in 0..NUM_MOTORS {
            write!(log_file, ",{:.6}", motor_state.positions[i] - DEFAULT_POSITION[i])?;
        }

        // Joint velocities
        for i in 0..NUM_MOTORS {
            write!(log_file, ",{:.6}", motor_state.velocities[i])?;
        }

        // Actions
        for i in 0..NUM_MOTORS {
            write!(log_file, ",{:.6}", action[i])?;
        }

        // Targets
        for i in 0..NUM_MOTORS {
            write!(log_file, ",{:.6}", targets[i])?;
        }

        writeln!(log_file)?;

        last_action = action;
        sample_count += 1;

        // Print progress
        if sample_count % 25 == 0 {
            println!("Logged {} samples ({:.1}s)...", sample_count, t);
        }

        // Maintain loop rate (50 Hz)
        let elapsed = iter_start.elapsed();
        if elapsed < Duration::from_millis(20) {
            thread::sleep(Duration::from_millis(20) - elapsed);
        }
    }

    println!();
    println!("✓ Logging complete!");
    println!("Wrote {} samples to {}", sample_count, args.output);
    println!();
    println!("Analysis tips:");
    println!("  - Check if action values are reasonable (typically < 1.0 rad)");
    println!("  - Look for oscillations in joint positions");
    println!("  - Verify projected gravity looks correct");
    println!("  - Check if velocities are reasonable");

    Ok(())
}
