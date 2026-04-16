use anyhow::Result;
use clap::Parser;
use microduck_runtime::imu::{ImuController, Bno08xController, Bmi088Controller, AnyImuController};
use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::thread;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(about = "Test IMU sensor readings (gyro + projected gravity), with optional digital-twin streaming")]
struct Args {
    /// Use BNO08X IMU (BNO080/085/086) instead of the default BNO055
    #[arg(long)]
    bno08x: bool,

    /// Use BMI088 IMU instead of the default BNO055
    #[arg(long)]
    bmi088: bool,

    /// Use projected gravity from quaternion instead of raw accelerometer (BNO055 only)
    #[arg(long)]
    projected_gravity: bool,

    /// Apply fold-robot axis remap after reading:
    ///   physical (x=up, y=left, z=back) → standard (x=forward, y=left, z=up)
    ///   accel: new = [-old_z, old_y, old_x]   gyro: new = [-old_gz, old_gy, old_gx]
    #[arg(long)]
    fold: bool,

    /// Stream IMU state over TCP in the same digital-twin format as the main runtime.
    /// Packet: 36 × f32 LE — [qw,qx,qy,qz] + [15 joint pos = 0.0] + [15 currents = 0.0] + [odo_x=0, odo_y=0]
    #[arg(long)]
    stream: bool,

    /// TCP port for digital-twin streaming (default: 9870)
    #[arg(long, default_value_t = 9870)]
    stream_port: u16,

    /// Output rate in Hz
    #[arg(long, default_value_t = 50)]
    hz: u32,
}

fn send_stream_packet(client: &mut TcpStream, quat: [f64; 4]) -> bool {
    let mut buf = [0u8; 36 * 4];
    let floats: [f32; 36] = [
        // [0..3]  IMU quaternion [w, x, y, z]
        quat[0] as f32, quat[1] as f32, quat[2] as f32, quat[3] as f32,
        // [4..18] joint positions — all zero (no motors in test_imu)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        // [19..33] currents in mA — all zero
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        // [34..35] odometry x, y — zero
        0.0, 0.0,
    ];
    for (i, f) in floats.iter().enumerate() {
        buf[i * 4..(i + 1) * 4].copy_from_slice(&f.to_le_bytes());
    }
    client.write_all(&buf).is_ok()
}

fn main() -> Result<()> {
    let args = Args::parse();

    let imu_name = if args.bno08x { "BNO08X" } else if args.bmi088 { "BMI088" } else { "BNO055" };
    println!("╔════════════════════════════════════════════════════════════════╗");
    println!("║              {} IMU Test{}                            ║",
             imu_name, if args.fold { " [fold remap]" } else { "             " });
    println!("╚════════════════════════════════════════════════════════════════╝");
    println!();

    let mut imu = if args.bno08x {
        println!("Initializing BNO08X on /dev/i2c-1 at 0x4B...");
        let ctrl = Bno08xController::new_default()
            .map_err(|e| anyhow::anyhow!("Failed to init BNO08X: {}", e))?;
        println!("✓ BNO08X initialized (rotation vector fusion)");
        AnyImuController::Bno08x(ctrl)
    } else if args.bmi088 {
        println!("Initializing BMI088 on /dev/i2c-1...");
        let ctrl = Bmi088Controller::new_default()
            .map_err(|e| anyhow::anyhow!("Failed to init BMI088: {}", e))?;
        println!("✓ BMI088 initialized (Madgwick filter)");
        AnyImuController::Bmi088(ctrl)
    } else {
        println!("Initializing BNO055 on /dev/i2c-1 at 0x29...");
        let ctrl = ImuController::new_default_with_mode(args.projected_gravity)
            .map_err(|e| anyhow::anyhow!("Failed to init BNO055: {}", e))?;
        if args.projected_gravity {
            println!("✓ BNO055 initialized (projected gravity from quaternion)");
        } else {
            println!("✓ BNO055 initialized (raw accelerometer)");
        }
        AnyImuController::Bno055(ctrl)
    };

    if args.fold {
        println!("✓ Fold axis remap enabled: (x=up,z=back) → (x=fwd,z=up)");
    }

    // Set up TCP streaming
    let listener = if args.stream {
        println!("✓ Streaming on TCP port {} (same format as --stream)", args.stream_port);
        let l = TcpListener::bind(("0.0.0.0", args.stream_port))
            .map_err(|e| anyhow::anyhow!("Failed to bind port {}: {}", args.stream_port, e))?;
        l.set_nonblocking(true)?;
        println!("  Waiting for digital-twin client...");
        Some(l)
    } else {
        None
    };

    println!();
    println!("Reading at {} Hz. Ctrl+C to stop.", args.hz);
    println!();
    println!("{:>10} {:>10} {:>10} | {:>10} {:>10} {:>10} | {:>10} {:>10} {:>10} {:>10}",
             "Gyro_X", "Gyro_Y", "Gyro_Z", "Grav_X", "Grav_Y", "Grav_Z",
             "Quat_W", "Quat_X", "Quat_Y", "Quat_Z");
    println!("{:>10} {:>10} {:>10} | {:>10} {:>10} {:>10} | {:>10} {:>10} {:>10} {:>10}",
             "(rad/s)", "(rad/s)", "(rad/s)", "(unit)", "(unit)", "(unit)",
             "", "", "", "");
    println!("{}", "=".repeat(115));

    let period = Duration::from_micros(1_000_000 / args.hz as u64);
    let mut stream_client: Option<TcpStream> = None;
    let mut last_print = Instant::now();
    let print_every = Duration::from_millis(100); // cap console at ~10 Hz regardless of IMU rate

    loop {
        let step_start = Instant::now();

        // Accept a new streaming client if we don't have one
        if let Some(ref l) = listener {
            if stream_client.is_none() {
                if let Ok((s, addr)) = l.accept() {
                    let _ = s.set_nonblocking(true);
                    let _ = s.set_nodelay(true);
                    println!("Digital-twin client connected: {}", addr);
                    stream_client = Some(s);
                }
            }
        }

        match imu.read() {
            Ok(mut data) => {
                // Apply fold axis remap if requested (identical to main runtime).
                // Physical fold IMU: x=up, y=left, z=back → standard: x=fwd, y=left, z=up
                if args.fold {
                    let [ax, ay, az] = data.accel;
                    let [gx, gy, gz] = data.gyro;
                    data.accel = [-az, ay, ax];
                    data.gyro  = [-gz, gy, gx];
                    // Quaternion correction: post-multiply by +90° around Y = [√2/2, 0, √2/2, 0]
                    // Derived from the same remap: BNO frame is at -90° around Y from standard,
                    // so q_std = q_bno * Rot_Y(+90°).
                    // q_bno * [w2=s, x2=0, y2=s, z2=0]:
                    //   w_new = (qw - qy) * s
                    //   x_new = (qx - qz) * s
                    //   y_new = (qw + qy) * s
                    //   z_new = (qx + qz) * s
                    let s = std::f64::consts::FRAC_1_SQRT_2;
                    let [qw, qx, qy, qz] = data.quat;
                    data.quat = [
                        (qw - qy) * s,
                        (qx - qz) * s,
                        (qw + qy) * s,
                        (qx + qz) * s,
                    ];
                }

                // Stream to digital-twin client
                if let Some(ref mut client) = stream_client {
                    if !send_stream_packet(client, data.quat) {
                        println!("Digital-twin client disconnected");
                        stream_client = None;
                    }
                }

                // Print to console (throttled)
                if last_print.elapsed() >= print_every {
                    last_print = Instant::now();
                    println!("{:10.4} {:10.4} {:10.4} | {:10.4} {:10.4} {:10.4} | {:10.4} {:10.4} {:10.4} {:10.4}",
                             data.gyro[0], data.gyro[1], data.gyro[2],
                             data.accel[0], data.accel[1], data.accel[2],
                             data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
                }
            }
            Err(e) => {
                eprintln!("Read error: {}", e);
            }
        }

        // Maintain target rate
        let elapsed = step_start.elapsed();
        if elapsed < period {
            thread::sleep(period - elapsed);
        }
    }
}
