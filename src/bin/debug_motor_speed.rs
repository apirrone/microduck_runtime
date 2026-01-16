use anyhow::{Context, Result};
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::f64::consts::PI;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};
use serialport;

const LEFT_ANKLE_ID: u8 = 24;
const RADS_PER_SEC_PER_COUNT: f64 = 0.229 * (2.0 * PI / 60.0);

fn main() -> Result<()> {
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║         Motor Speed Unit Verification Tool                    ║");
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("This tool commands a sinusoidal motion on the left ankle joint");
    println!("and verifies that the velocity units are correct by comparing:");
    println!("  - Theoretical max velocity (from sine wave derivative)");
    println!("  - Measured max velocity (from motor feedback)");
    println!();
    println!("⚠️  WARNING: Ensure the robot's left leg can move freely!");
    println!("    The left ankle will oscillate ±0.3 rad at 1 Hz");
    println!();

    // Get serial port from args or use default
    let port = std::env::args().nth(1).unwrap_or("/dev/ttyUSB0".to_string());
    let baudrate = 1_000_000;

    println!("Connecting to motors on {} at {} baud...", port, baudrate);

    // Open serial port
    let serial_port = serialport::new(&port, baudrate)
        .timeout(Duration::from_millis(100))
        .open()
        .context("Failed to open serial port")?;

    // Create controller
    let mut controller = Xl330Controller::new()
        .with_protocol_v2()
        .with_serial_port(serial_port);

    println!("✓ Connected to motors");
    println!();

    // Read initial position
    println!("Reading current left ankle position...");
    let initial_position = controller
        .read_present_position(LEFT_ANKLE_ID)
        .context("Failed to read initial position")?;
    println!("✓ Current position: {:.3} rad", initial_position);
    println!();

    // Enable torque
    println!("Enabling torque on left ankle (ID {})...", LEFT_ANKLE_ID);
    controller
        .write_torque_enable(LEFT_ANKLE_ID, true)
        .context("Failed to enable torque")?;
    println!("✓ Torque enabled");
    println!();

    // Set moderate PID gains
    println!("Setting PID gains...");
    controller
        .write_position_p_gain(LEFT_ANKLE_ID, 400)
        .context("Failed to set P gain")?;
    controller
        .write_position_i_gain(LEFT_ANKLE_ID, 0)
        .context("Failed to set I gain")?;
    controller
        .write_position_d_gain(LEFT_ANKLE_ID, 0)
        .context("Failed to set D gain")?;
    println!("✓ PID gains set (P=400, I=0, D=0)");
    println!();

    // Sine wave parameters
    let center = initial_position;
    let amplitude = 0.3; // rad (about 17 degrees)
    let frequency = 1.0; // Hz
    let duration_sec = 10.0; // seconds

    // Calculate theoretical max velocity
    // For position = center + amplitude * sin(2π * frequency * t)
    // velocity = amplitude * 2π * frequency * cos(2π * frequency * t)
    // max velocity = amplitude * 2π * frequency
    let theoretical_max_velocity = amplitude * 2.0 * PI * frequency;

    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║ Sine Wave Parameters                                           ║");
    println!("╠════════════════════════════════════════════════════════════════╣");
    println!("║ Center position:     {:.3} rad                               ║", center);
    println!("║ Amplitude:           {:.3} rad                               ║", amplitude);
    println!("║ Frequency:           {:.1} Hz                                 ║", frequency);
    println!("║ Duration:            {:.1} seconds                            ║", duration_sec);
    println!("║                                                                ║");
    println!("║ Theoretical max velocity: {:.3} rad/s                        ║", theoretical_max_velocity);
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();

    println!("Press Enter to start the motion test...");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input)?;

    println!();
    println!("Starting sinusoidal motion test...");
    println!();
    println!("┌────────────────────────────────────────────────────────────────┐");
    println!("│  Time    Goal Pos   Actual Pos   Goal Vel   Actual Vel         │");
    println!("│  (s)     (rad)      (rad)        (rad/s)    (rad/s)             │");
    println!("├────────────────────────────────────────────────────────────────┤");

    let start_time = Instant::now();
    let loop_period = Duration::from_millis(20); // 50 Hz
    let mut max_measured_velocity = 0.0_f64;

    loop {
        let iteration_start = Instant::now();
        let elapsed = start_time.elapsed().as_secs_f64();

        if elapsed > duration_sec {
            break;
        }

        // Calculate goal position and theoretical velocity
        let t = elapsed;
        let goal_position = center + amplitude * (2.0 * PI * frequency * t).sin();
        let theoretical_velocity = amplitude * 2.0 * PI * frequency * (2.0 * PI * frequency * t).cos();

        // Command goal position
        controller
            .write_goal_position(LEFT_ANKLE_ID, goal_position)
            .context("Failed to write goal position")?;

        // Read actual position and velocity
        let actual_position = controller
            .read_present_position(LEFT_ANKLE_ID)
            .unwrap_or(0.0);

        let raw_velocity = controller
            .read_present_velocity(LEFT_ANKLE_ID)
            .unwrap_or(0) as f64;

        let actual_velocity = raw_velocity * RADS_PER_SEC_PER_COUNT;

        // Track max measured velocity
        max_measured_velocity = max_measured_velocity.max(actual_velocity.abs());

        // Print status every 0.1 seconds (5 iterations at 50 Hz)
        if (elapsed * 10.0) as u32 % 1 == 0 {
            print!("│ {:5.2}    {:7.3}     {:7.3}      {:7.3}     {:7.3}          │\r",
                   elapsed, goal_position, actual_position, theoretical_velocity, actual_velocity);
            std::io::stdout().flush()?;
        }

        // Sleep to maintain loop rate
        let elapsed_iteration = iteration_start.elapsed();
        if elapsed_iteration < loop_period {
            thread::sleep(loop_period - elapsed_iteration);
        }
    }

    println!();
    println!("└────────────────────────────────────────────────────────────────┘");
    println!();

    // Return to initial position
    println!("Returning to initial position...");
    controller
        .write_goal_position(LEFT_ANKLE_ID, initial_position)
        .context("Failed to write return position")?;
    thread::sleep(Duration::from_millis(500));

    // Disable torque
    println!("Disabling torque...");
    controller
        .write_torque_enable(LEFT_ANKLE_ID, false)
        .context("Failed to disable torque")?;
    println!("✓ Torque disabled");
    println!();

    // Display results
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║ Results                                                        ║");
    println!("╠════════════════════════════════════════════════════════════════╣");
    println!("║ Theoretical max velocity: {:.3} rad/s                        ║", theoretical_max_velocity);
    println!("║ Measured max velocity:    {:.3} rad/s                        ║", max_measured_velocity);
    println!("║                                                                ║");

    let velocity_ratio = if theoretical_max_velocity != 0.0 {
        max_measured_velocity / theoretical_max_velocity
    } else {
        0.0
    };

    let velocity_error = (max_measured_velocity - theoretical_max_velocity).abs();
    let velocity_error_pct = if theoretical_max_velocity != 0.0 {
        100.0 * velocity_error / theoretical_max_velocity
    } else {
        0.0
    };

    println!("║ Ratio (measured/theory):  {:.3}                               ║", velocity_ratio);
    println!("║ Error:                    {:.3} rad/s ({:.1}%)              ║",
             velocity_error, velocity_error_pct);
    println!("║                                                                ║");

    if velocity_error_pct < 10.0 {
        println!("║ ✓ PASS: Velocity units appear correct                         ║");
    } else if velocity_error_pct < 30.0 {
        println!("║ ⚠ WARNING: Velocity has {:.1}% error                         ║", velocity_error_pct);
        println!("║   This might be acceptable depending on load/PID tuning       ║");
    } else {
        println!("║ ✗ FAIL: Velocity units may be incorrect                       ║");
        println!("║   Expected factor of 1.0, got {:.3}                          ║", velocity_ratio);
        if velocity_ratio > 1.5 || velocity_ratio < 0.5 {
            println!("║   Consider checking RADS_PER_SEC_PER_COUNT in motor.rs        ║");
        }
    }

    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();
    println!("Note: Some error is expected due to:");
    println!("  - Motor PID tracking lag");
    println!("  - Load on the joint");
    println!("  - Measurement timing");
    println!();
    println!("Usage:");
    println!("  debug_motor_speed [PORT]");
    println!("  debug_motor_speed /dev/ttyUSB0");

    Ok(())
}
