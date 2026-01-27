use anyhow::{Context, Result};
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use serialport;
use std::f64::consts::PI;
use std::io::{self, Write as IoWrite};
use std::time::{Duration, Instant};

const TEST_MOTOR_ID: u8 = 24; // Left ankle motor
const DEFAULT_POSITION: f64 = 0.6; // Default ankle position (radians)
const AMPLITUDE: f64 = 45.0 * PI / 180.0; // ±45° amplitude
const SAMPLE_RATE_HZ: u32 = 200; // Read position at 200 Hz
const POSITION_THRESHOLD: f64 = 0.05; // 0.05 rad (~3°) to detect movement

// Test frequencies in Hz
const TEST_FREQUENCIES: [f64; 5] = [0.5, 1.0, 2.0, 3.0, 5.0];

fn main() -> Result<()> {
    println!("=== Motor Command Latency Test ===\n");
    println!("This test measures the delay between sending a position command");
    println!("and the motor actually starting to move.\n");

    // Get serial port
    println!("Available serial ports:");
    let ports = serialport::available_ports()?;
    for (i, port) in ports.iter().enumerate() {
        println!("  [{}] {}", i, port.port_name);
    }

    let port = if ports.is_empty() {
        println!("\n⚠️  No serial ports found. Defaulting to /dev/ttyUSB0");
        "/dev/ttyUSB0".to_string()
    } else if ports.len() == 1 {
        println!("\nUsing: {}", ports[0].port_name);
        ports[0].port_name.clone()
    } else {
        print!("\nEnter port number [0]: ");
        io::stdout().flush()?;
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let idx: usize = input.trim().parse().unwrap_or(0);
        ports.get(idx).map(|p| p.port_name.clone()).unwrap_or_else(|| ports[0].port_name.clone())
    };

    println!("\nInitializing motor controller...");
    let baudrate = 1_000_000; // 1M baud
    let serial_port = serialport::new(&port, baudrate)
        .timeout(Duration::from_millis(500))
        .open()
        .context("Failed to open serial port")?;

    let mut controller = Xl330Controller::new()
        .with_protocol_v2()
        .with_serial_port(serial_port);

    // Test motor connectivity
    print!("Testing connection to motor {}...", TEST_MOTOR_ID);
    io::stdout().flush()?;
    match controller.ping(TEST_MOTOR_ID) {
        Ok(true) => println!(" ✓"),
        Ok(false) => {
            println!(" ✗");
            anyhow::bail!("Motor {} did not respond to ping", TEST_MOTOR_ID);
        }
        Err(e) => {
            println!(" ✗");
            anyhow::bail!("Failed to ping motor {}: {}", TEST_MOTOR_ID, e);
        }
    }

    // Read current position
    let current_pos_raw = controller.read_present_position(TEST_MOTOR_ID)
        .map_err(|e| anyhow::anyhow!("Failed to read current position: {}", e))?;
    let current_pos = current_pos_raw[0];
    println!("Current position: {:.3} rad ({:.1}°)", current_pos, current_pos * 180.0 / PI);

    // Safety check
    println!("\n╔════════════════════════════════════════════════════════╗");
    println!("║  SAFETY WARNINGS:                                      ║");
    println!("║  - This will move the LEFT ANKLE motor                ║");
    println!("║  - Amplitude: ±45° around default position            ║");
    println!("║  - Range: {:.1}° to {:.1}°                              ║",
             ((DEFAULT_POSITION - AMPLITUDE) * 180.0 / PI),
             ((DEFAULT_POSITION + AMPLITUDE) * 180.0 / PI));
    println!("║  - Make sure robot is stable and motor can move freely ║");
    println!("╚════════════════════════════════════════════════════════╝\n");

    print!("Press ENTER to start test (or Ctrl+C to abort)...");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    // Enable torque
    println!("\nEnabling torque...");
    controller.write_torque_enable(TEST_MOTOR_ID, true)
        .map_err(|e| anyhow::anyhow!("Failed to enable torque: {}", e))?;

    // Move to default position
    println!("Moving to default position ({:.1}°)...", DEFAULT_POSITION * 180.0 / PI);
    controller.write_goal_position(TEST_MOTOR_ID, DEFAULT_POSITION)
        .map_err(|e| anyhow::anyhow!("Failed to set goal position: {}", e))?;
    std::thread::sleep(Duration::from_secs(2));

    println!("\nStarting latency tests...\n");
    println!("═══════════════════════════════════════════════════════");

    let mut results = Vec::new();

    for (test_num, &frequency) in TEST_FREQUENCIES.iter().enumerate() {
        println!("\nTest {}/{}: {:.1} Hz sine wave", test_num + 1, TEST_FREQUENCIES.len(), frequency);
        println!("───────────────────────────────────────────────────────");

        let period = 1.0 / frequency;
        let num_cycles = 3; // Test 3 complete cycles
        let test_duration = Duration::from_secs_f64(num_cycles as f64 * period);

        let mut cycle_latencies = Vec::new();

        for cycle in 0..num_cycles {
            println!("  Cycle {}/{}...", cycle + 1, num_cycles);

            // Wait at center position
            std::thread::sleep(Duration::from_millis(200));

            // Calculate target position (move to peak of sine wave)
            let target_pos = DEFAULT_POSITION + AMPLITUDE;

            // Get baseline position
            let baseline_raw = controller.read_present_position(TEST_MOTOR_ID)
                .map_err(|e| anyhow::anyhow!("Failed to read baseline position: {}", e))?;
            let baseline_pos = baseline_raw[0];

            // Send command and start timing
            let command_time = Instant::now();
            controller.write_goal_position(TEST_MOTOR_ID, target_pos)
                .map_err(|e| anyhow::anyhow!("Failed to write goal position: {}", e))?;

            // Immediately start sampling position at high rate
            let dt = Duration::from_micros(1_000_000 / SAMPLE_RATE_HZ as u64);
            let mut last_sample = Instant::now();
            let mut movement_detected = false;
            let mut latency = Duration::ZERO;
            let mut samples = Vec::new();

            // Sample for up to half the period (should detect movement much sooner)
            let max_sample_time = Duration::from_secs_f64(period / 2.0);

            while command_time.elapsed() < max_sample_time {
                let now = Instant::now();

                if now.duration_since(last_sample) >= dt {
                    match controller.read_present_position(TEST_MOTOR_ID) {
                        Ok(pos_raw) => {
                            let pos = pos_raw[0];
                            let timestamp = command_time.elapsed();
                            let deviation = (pos - baseline_pos).abs();
                            samples.push((timestamp, pos, deviation));

                            // Detect movement
                            if !movement_detected && deviation > POSITION_THRESHOLD {
                                movement_detected = true;
                                latency = timestamp;
                            }

                            last_sample = now;
                        }
                        Err(e) => {
                            eprintln!("    Warning: Failed to read position: {}", e);
                        }
                    }
                }

                std::thread::sleep(Duration::from_micros(100));
            }

            if movement_detected {
                let latency_ms = latency.as_secs_f64() * 1000.0;
                println!("    ✓ Movement detected: {:.1} ms", latency_ms);
                cycle_latencies.push(latency_ms);
            } else {
                println!("    ⚠️  No movement detected (motor may be stalled)");
            }

            // Move back to center for next cycle
            controller.write_goal_position(TEST_MOTOR_ID, DEFAULT_POSITION)
                .map_err(|e| anyhow::anyhow!("Failed to return to center: {}", e))?;
            std::thread::sleep(Duration::from_millis((period * 500.0) as u64));
        }

        // Calculate statistics for this frequency
        if !cycle_latencies.is_empty() {
            let mean = cycle_latencies.iter().sum::<f64>() / cycle_latencies.len() as f64;
            let variance = cycle_latencies.iter()
                .map(|&x| (x - mean).powi(2))
                .sum::<f64>() / cycle_latencies.len() as f64;
            let std_dev = variance.sqrt();
            let min = cycle_latencies.iter().cloned().fold(f64::INFINITY, f64::min);
            let max = cycle_latencies.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

            println!("\n  Results for {:.1} Hz:", frequency);
            println!("    Mean:   {:.1} ms", mean);
            println!("    Std:    {:.1} ms", std_dev);
            println!("    Range:  {:.1} - {:.1} ms", min, max);

            results.push((frequency, mean, std_dev, min, max));
        } else {
            println!("\n  ⚠️  No valid measurements for {:.1} Hz", frequency);
        }
    }

    // Return to default position
    println!("\n═══════════════════════════════════════════════════════");
    println!("\nReturning to default position...");
    controller.write_goal_position(TEST_MOTOR_ID, DEFAULT_POSITION)
        .map_err(|e| anyhow::anyhow!("Failed to return to default: {}", e))?;
    std::thread::sleep(Duration::from_secs(1));

    // Disable torque
    println!("Disabling torque...");
    controller.write_torque_enable(TEST_MOTOR_ID, false)
        .map_err(|e| anyhow::anyhow!("Failed to disable torque: {}", e))?;

    // Print summary
    println!("\n═══════════════════════════════════════════════════════");
    println!("                    SUMMARY                            ");
    println!("═══════════════════════════════════════════════════════");
    println!("\nFreq(Hz) | Mean(ms) | Std(ms) | Min(ms) | Max(ms)");
    println!("---------|----------|---------|---------|----------");

    for (freq, mean, std, min, max) in &results {
        println!("  {:5.1}  |  {:6.1}  |  {:5.1}  |  {:5.1}  |  {:6.1}",
                 freq, mean, std, min, max);
    }

    println!("\n💡 Interpretation:");
    println!("   - Mean latency: Time from command to first detectable movement");
    println!("   - Lower is better (typical: 5-20ms)");
    println!("   - Higher frequencies may show increased latency due to:");
    println!("     * Motor acceleration limits");
    println!("     * Communication overhead");
    println!("     * PID controller response time");
    println!("\n   For 50Hz control (20ms period):");
    println!("   - Motor latency should be < 15ms for responsive control");
    println!("   - Total loop latency = IMU read + inference + motor latency");

    Ok(())
}
