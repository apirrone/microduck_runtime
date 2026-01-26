use anyhow::{Context, Result};
use microduck_runtime::imu::ImuController;
use std::io::{self, Write};
use std::time::{Duration, Instant};

const SAMPLE_RATE_HZ: u32 = 200; // Sample at 200 Hz
const BASELINE_SAMPLES: usize = 20; // Samples to establish baseline
const POST_TRIGGER_MS: u64 = 500; // Continue reading for 500ms after trigger

fn main() -> Result<()> {
    println!("=== IMU Fusion Latency Test (Projected Gravity) ===\n");
    println!("This test measures the latency of the FUSED sensor output");
    println!("(what your control loop actually uses)\n");

    // Initialize IMU
    println!("Initializing IMU (BNO055 on I2C)...");
    let mut imu = ImuController::new_default()
        .context("Failed to initialize IMU")?;
    println!("✓ IMU initialized\n");

    // Calculate baseline for both raw and fused data
    println!("Establishing baseline (keep robot still)...");
    let mut baseline_raw_accel = [0.0; 3];
    let mut baseline_raw_gyro = [0.0; 3];
    let mut baseline_projected_gravity = [0.0; 3];

    for _ in 0..BASELINE_SAMPLES {
        let data = imu.read().context("Failed to read IMU")?;
        for i in 0..3 {
            baseline_raw_accel[i] += data.accel[i];
            baseline_raw_gyro[i] += data.gyro[i];
            baseline_projected_gravity[i] += data.accel[i]; // accel is already projected gravity
        }
        std::thread::sleep(Duration::from_millis(5));
    }

    // Average the baseline
    for i in 0..3 {
        baseline_raw_accel[i] /= BASELINE_SAMPLES as f64;
        baseline_raw_gyro[i] /= BASELINE_SAMPLES as f64;
        baseline_projected_gravity[i] /= BASELINE_SAMPLES as f64;
    }

    println!("✓ Baseline established:");
    println!("  Raw Accel:          [{:.3}, {:.3}, {:.3}]",
             baseline_raw_accel[0], baseline_raw_accel[1], baseline_raw_accel[2]);
    println!("  Raw Gyro:           [{:.3}, {:.3}, {:.3}] rad/s",
             baseline_raw_gyro[0], baseline_raw_gyro[1], baseline_raw_gyro[2]);
    println!("  Projected Gravity:  [{:.3}, {:.3}, {:.3}]",
             baseline_projected_gravity[0], baseline_projected_gravity[1], baseline_projected_gravity[2]);

    // Check if projected gravity looks reasonable (should be close to unit vector pointing down)
    let gravity_mag = (baseline_projected_gravity[0].powi(2) +
                       baseline_projected_gravity[1].powi(2) +
                       baseline_projected_gravity[2].powi(2)).sqrt();
    println!("  Gravity magnitude:  {:.3} (should be ~1.0)\n", gravity_mag);

    if (gravity_mag - 1.0).abs() > 0.2 {
        println!("⚠️  Warning: Gravity magnitude is not close to 1.0");
        println!("   This may indicate IMU calibration issues or incorrect axis mapping\n");
    }

    // Instructions
    println!("╔════════════════════════════════════════════════════════╗");
    println!("║  INSTRUCTIONS:                                         ║");
    println!("║  1. Keep robot in stable init position                ║");
    println!("║  2. Press ENTER and immediately TILT/ROTATE robot     ║");
    println!("║     (change orientation, not just bump)                ║");
    println!("║  3. We'll compare raw sensor vs fused gravity latency ║");
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

                    // Calculate deviation from baseline for raw gyro
                    let gyro_dev: f64 = (0..3)
                        .map(|i| (data.gyro[i] - baseline_raw_gyro[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();

                    // Calculate deviation from baseline for projected gravity (what control loop uses)
                    let gravity_dev: f64 = (0..3)
                        .map(|i| (data.accel[i] - baseline_projected_gravity[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();

                    samples.push((timestamp, gyro_dev, gravity_dev));
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
    const GYRO_THRESHOLD: f64 = 0.05;     // 0.05 rad/s deviation (~3°/s)
    const GRAVITY_THRESHOLD: f64 = 0.05;  // 0.05g change in projected gravity

    // Find first significant change in gyro (raw sensor)
    let gyro_detection = samples.iter()
        .enumerate()
        .find(|(_, (_, gyro_dev, _))| *gyro_dev > GYRO_THRESHOLD);

    // Find first significant change in projected gravity (fused output)
    let gravity_detection = samples.iter()
        .enumerate()
        .find(|(_, (_, _, gravity_dev))| *gravity_dev > GRAVITY_THRESHOLD);

    // Print results
    println!("═══════════════════════════════════════════════════════");
    println!("              FUSION LATENCY RESULTS                   ");
    println!("═══════════════════════════════════════════════════════");

    let gyro_time = gyro_detection.map(|(_, (t, _, _))| t.as_secs_f64() * 1000.0);
    let gravity_time = gravity_detection.map(|(_, (t, _, _))| t.as_secs_f64() * 1000.0);

    match gyro_detection {
        Some((_, (t, gyro_dev, _))) => {
            println!("📊 Raw Gyroscope detected movement:");
            println!("   Latency: {:.1} ms", t.as_secs_f64() * 1000.0);
        }
        None => {
            println!("⚠️  Raw Gyroscope: No movement detected");
            println!("   (threshold: {:.3} rad/s = {:.1}°/s)", GYRO_THRESHOLD, GYRO_THRESHOLD.to_degrees());
        }
    }

    println!();

    match gravity_detection {
        Some((_, (t, _, gravity_dev))) => {
            println!("📊 Projected Gravity (FUSED) detected change:");
            println!("   Latency: {:.1} ms", t.as_secs_f64() * 1000.0);
        }
        None => {
            println!("⚠️  Projected Gravity: No change detected");
            println!("   (threshold: {:.3}g)", GRAVITY_THRESHOLD);
        }
    }

    println!();

    // Compare latencies
    if let (Some(gyro_t), Some(grav_t)) = (gyro_time, gravity_time) {
        let diff = grav_t - gyro_t;
        println!("⏱️  Fusion Algorithm Delay:");
        if diff > 0.0 {
            println!("   Projected gravity lagged raw gyro by {:.1} ms", diff);
            if diff > 10.0 {
                println!("   ⚠️  Fusion delay is significant (>10ms)");
            } else if diff > 5.0 {
                println!("   ℹ️  Moderate fusion delay (5-10ms)");
            } else {
                println!("   ✅ Minimal fusion delay (<5ms)");
            }
        } else {
            println!("   Projected gravity detected BEFORE raw gyro by {:.1} ms", -diff);
            println!("   ℹ️  This can happen if tilt is more detectable than rotation rate");
        }
    }

    println!("═══════════════════════════════════════════════════════\n");

    // Show detailed timeline
    println!("Detailed timeline (first 50ms):");
    println!("Time(ms)  | Raw Gyro  | Proj.Grav | Status");
    println!("----------|-----------|-----------|------------------");

    for (timestamp, gyro_dev, gravity_dev) in samples.iter().take(100) {
        let time_ms = timestamp.as_secs_f64() * 1000.0;
        if time_ms > 50.0 {
            break;
        }

        let mut status = String::new();
        if *gyro_dev > GYRO_THRESHOLD {
            status.push_str("GYRO ");
        }
        if *gravity_dev > GRAVITY_THRESHOLD {
            status.push_str("GRAVITY ");
        }

        println!("{:8.1}  | {:9.4} | {:9.4} | {}",
                 time_ms, gyro_dev, gravity_dev, status);
    }

    println!("\n💡 Interpretation:");
    println!("   - Raw Gyro: Direct sensor measurement (minimal latency)");
    println!("   - Projected Gravity: Sensor fusion output (used by control loop)");
    println!("   - Fusion delay < 5ms: Excellent, fusion is fast");
    println!("   - Fusion delay > 10ms: May impact high-frequency control");
    println!("\n   For 50Hz control (20ms period):");
    println!("   - Total IMU latency should be < 15ms for good performance");
    println!("   - Your control loop uses the FUSED output (projected gravity)");

    Ok(())
}
