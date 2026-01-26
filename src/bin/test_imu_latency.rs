use anyhow::{Context, Result};
use microduck_runtime::imu::ImuController;
use std::io::{self, Write};
use std::time::{Duration, Instant};

const SAMPLE_RATE_HZ: u32 = 200; // Sample at 200 Hz
const BASELINE_SAMPLES: usize = 20; // Samples to establish baseline
const POST_TRIGGER_MS: u64 = 500; // Continue reading for 500ms after trigger

fn main() -> Result<()> {
    println!("=== IMU Latency Test ===\n");

    // Initialize IMU
    println!("Initializing IMU (BNO055 on I2C)...");
    let mut imu = ImuController::new_default()
        .context("Failed to initialize IMU")?;
    println!("✓ IMU initialized\n");

    // Calculate baseline
    println!("Establishing baseline (keep robot still)...");
    let mut baseline_accel = [0.0; 3];
    let mut baseline_gyro = [0.0; 3];

    for _ in 0..BASELINE_SAMPLES {
        let data = imu.read().context("Failed to read IMU")?;
        for i in 0..3 {
            baseline_accel[i] += data.accel[i];
            baseline_gyro[i] += data.gyro[i];
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    // Average the baseline
    for i in 0..3 {
        baseline_accel[i] /= BASELINE_SAMPLES as f64;
        baseline_gyro[i] /= BASELINE_SAMPLES as f64;
    }

    println!("✓ Baseline established:");
    println!("  Accel: [{:.3}, {:.3}, {:.3}]",
             baseline_accel[0], baseline_accel[1], baseline_accel[2]);
    println!("  Gyro:  [{:.3}, {:.3}, {:.3}] rad/s\n",
             baseline_gyro[0], baseline_gyro[1], baseline_gyro[2]);

    // Instructions
    println!("╔════════════════════════════════════════════════════════╗");
    println!("║  INSTRUCTIONS:                                         ║");
    println!("║  1. Keep robot in stable init position                ║");
    println!("║  2. When ready, press ENTER and immediately bump/tap  ║");
    println!("║     the robot (try to do both at the same time)       ║");
    println!("║  3. We'll measure time between Enter and IMU change   ║");
    println!("╚════════════════════════════════════════════════════════╝\n");

    print!("Press ENTER when ready...");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    // Start the test
    println!("\n🚀 TRIGGER! Sampling IMU...\n");
    let trigger_time = Instant::now();

    let dt = Duration::from_micros(1_000_000 / SAMPLE_RATE_HZ as u64);
    let mut samples = Vec::new();

    // Sample for POST_TRIGGER_MS milliseconds
    let test_duration = Duration::from_millis(POST_TRIGGER_MS);
    let mut last_sample = Instant::now();

    while trigger_time.elapsed() < test_duration {
        let now = Instant::now();

        // Try to maintain consistent sample rate
        if now.duration_since(last_sample) >= dt {
            match imu.read() {
                Ok(data) => {
                    let timestamp = trigger_time.elapsed();

                    // Calculate deviation from baseline
                    let accel_dev: f64 = (0..3)
                        .map(|i| (data.accel[i] - baseline_accel[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();

                    let gyro_dev: f64 = (0..3)
                        .map(|i| (data.gyro[i] - baseline_gyro[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();

                    samples.push((timestamp, accel_dev, gyro_dev));
                    last_sample = now;
                }
                Err(e) => {
                    eprintln!("Warning: Failed to read IMU: {}", e);
                }
            }
        }

        // Small sleep to avoid busy waiting
        std::thread::sleep(Duration::from_micros(100));
    }

    // Analyze results
    println!("✓ Collected {} samples\n", samples.len());
    println!("Analyzing data...\n");

    // Define thresholds for movement detection
    const ACCEL_THRESHOLD: f64 = 0.05; // 0.05g deviation (~0.5 m/s²)
    const GYRO_THRESHOLD: f64 = 0.05;  // 0.05 rad/s deviation (~3°/s) - lowered for sensitivity

    // Find first significant change in acceleration
    let accel_detection = samples.iter()
        .enumerate()
        .find(|(_, (_, accel_dev, _))| *accel_dev > ACCEL_THRESHOLD);

    // Find first significant change in gyro
    let gyro_detection = samples.iter()
        .enumerate()
        .find(|(_, (_, _, gyro_dev))| *gyro_dev > GYRO_THRESHOLD);

    // Print results
    println!("═══════════════════════════════════════════════════════");
    println!("                   RESULTS                             ");
    println!("═══════════════════════════════════════════════════════");

    match accel_detection {
        Some((idx, (t, accel_dev, gyro_dev))) => {
            println!("📊 Accelerometer detected movement:");
            println!("   Latency: {:.1} ms", t.as_secs_f64() * 1000.0);
            println!("   Gyro at this time: {:.4} rad/s ({:.1}°/s)",
                     gyro_dev, gyro_dev.to_degrees());
            if *gyro_dev < GYRO_THRESHOLD {
                println!("   ⚠️  Gyro below threshold - bump was mostly translational");
            }
        }
        None => {
            println!("⚠️  Accelerometer: No movement detected above threshold");
            println!("   (threshold: {:.3}g)", ACCEL_THRESHOLD);
        }
    }

    println!();

    match gyro_detection {
        Some((idx, (t, accel_dev, gyro_dev))) => {
            println!("📊 Gyroscope detected movement:");
            println!("   Latency: {:.1} ms", t.as_secs_f64() * 1000.0);
            if accel_detection.is_some() {
                let accel_idx = accel_detection.unwrap().0;
                if idx > accel_idx {
                    println!("   ⚠️  Gyro detected {:.1} ms AFTER accelerometer",
                             (t.as_secs_f64() - accel_detection.unwrap().1.0.as_secs_f64()) * 1000.0);
                }
            }
        }
        None => {
            println!("⚠️  Gyroscope: No movement detected above threshold");
            println!("   (threshold: {:.3} rad/s = {:.1}°/s)", GYRO_THRESHOLD, GYRO_THRESHOLD.to_degrees());
            println!("   💡 Bump was likely pure translation (no rotation)");
        }
    }

    println!("═══════════════════════════════════════════════════════\n");

    // Show detailed timeline for debugging
    println!("Detailed timeline (first 50ms):");
    println!("Time(ms)  | Accel Dev | Gyro Dev  | Status");
    println!("----------|-----------|-----------|------------------");

    for (timestamp, accel_dev, gyro_dev) in samples.iter().take(100) {
        let time_ms = timestamp.as_secs_f64() * 1000.0;
        if time_ms > 50.0 {
            break;
        }

        let mut status = String::new();
        if *accel_dev > ACCEL_THRESHOLD {
            status.push_str("ACCEL ");
        }
        if *gyro_dev > GYRO_THRESHOLD {
            status.push_str("GYRO ");
        }

        println!("{:8.1}  | {:9.4} | {:9.4} | {}",
                 time_ms, accel_dev, gyro_dev, status);
    }

    println!("\n💡 Tips:");
    println!("   - Lower latency = better responsiveness");
    println!("   - Typical BNO055 latency: 5-15ms for both sensors");
    println!("   - If latency > 20ms, check I2C speed and polling rate");
    println!("\n   Gyro vs Accel latency difference:");
    println!("   - If gyro >> accel: Bump was mostly translational (no rotation)");
    println!("   - Try tapping/twisting the robot to test gyro response");
    println!("   - For walking, both sensors matter equally");

    Ok(())
}
