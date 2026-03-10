/// Raw SHTP packet debugger for BNO08X
///
/// Stages:
///   1. Try reading raw packets (no init) to see boot messages
///   2. Send soft reset, drain boot packets
///   3. Send Set Feature Command for Game Rotation Vector (0x08)
///   4. Read and print all packets for 3 seconds, annotating known reports
///
/// Run with: sudo debug_bno08x

use anyhow::Result;
use embedded_hal::i2c::I2c;
use linux_embedded_hal::I2cdev;
use std::thread;
use std::time::{Duration, Instant};

const ADDR: u8 = 0x4B;
const I2C_DEV: &str = "/dev/i2c-1";

// SHTP channels
const CH_EXECUTABLE: u8 = 1;
const CH_CONTROL: u8 = 2;
const CH_REPORTS: u8 = 3;

fn channel_name(ch: u8) -> &'static str {
    match ch {
        0 => "SHTP_CMD",
        1 => "EXECUTABLE",
        2 => "CONTROL",
        3 => "SENSOR_REPORTS",
        4 => "WAKE_REPORTS",
        5 => "GYRO_RV",
        _ => "UNKNOWN",
    }
}

fn report_name(id: u8) -> &'static str {
    match id {
        0x01 => "Accelerometer",
        0x02 => "Gyro_Calibrated",
        0x03 => "MagField",
        0x04 => "LinearAccel",
        0x05 => "RotationVector",
        0x06 => "Gravity",
        0x08 => "GameRotationVector",
        0x09 => "GeomagRV",
        0xF1 => "GetFeatureResponse",
        0xF2 => "SetFeatureCommand",
        0xFB => "Timestamp",
        0xFC => "BaseTimestamp",
        0xFE => "FlushCompleted",
        _ => "?",
    }
}

fn hex(buf: &[u8]) -> String {
    buf.iter().map(|b| format!("{:02X}", b)).collect::<Vec<_>>().join(" ")
}

/// Try to read one SHTP packet using a single 300-byte I2C transaction.
/// The BNO08X advances its read pointer on every STOP, so one transaction = one packet.
/// Extra bytes beyond the packet are returned as 0xFF by the sensor (per SHTP spec).
fn try_read_packet(dev: &mut I2cdev) -> Option<(u8, Vec<u8>)> {
    let mut buf = [0u8; 300];
    if dev.read(ADDR, &mut buf).is_err() {
        return None; // NACK = no data
    }

    let total_len = ((buf[1] as usize & 0x7F) << 8) | buf[0] as usize;
    let channel = buf[2];
    let seq = buf[3];
    let continuation = buf[1] & 0x80 != 0;

    println!("  [PKT] raw_hdr={} total_len={} channel={} ({}) seq={} continuation={}",
             hex(&buf[0..4]), total_len, channel, channel_name(channel), seq, continuation);

    if total_len < 4 {
        println!("  [PKT] bad length, skipping");
        return None;
    }
    if total_len == 4 {
        println!("  [PKT] header-only (empty payload)");
        return Some((channel, vec![]));
    }

    let payload_end = total_len.min(buf.len());
    let payload = buf[4..payload_end].to_vec();
    Some((channel, payload))
}

fn write_packet(dev: &mut I2cdev, seq: &mut [u8; 8], channel: u8, payload: &[u8]) -> Result<()> {
    let total_len = (payload.len() + 4) as u16;
    let s = seq[channel as usize];
    seq[channel as usize] = s.wrapping_add(1);
    let mut buf = Vec::with_capacity(total_len as usize);
    buf.push(total_len as u8);
    buf.push((total_len >> 8) as u8);
    buf.push(channel);
    buf.push(s);
    buf.extend_from_slice(payload);
    println!("  [WRITE ch={} ({}) seq={}] {}", channel, channel_name(channel), s, hex(&buf));
    dev.write(ADDR, &buf)
        .map_err(|e| anyhow::anyhow!("I2C write failed: {:?}", e))?;
    Ok(())
}

fn annotate_sensor_report(payload: &[u8]) {
    // Channel 3 payload: [0]=timestamp_type [1..5]=timestamp [5]=report_id ...
    if payload.len() < 6 {
        println!("    payload too short to parse");
        return;
    }
    let ts_type = payload[0];
    let report_id = payload[5];
    println!("    ts_type=0x{:02X} report_id=0x{:02X} ({})", ts_type, report_id, report_name(report_id));

    if payload.len() >= 19 && (report_id == 0x05 || report_id == 0x08) {
        let i = i16::from_le_bytes([payload[9],  payload[10]]);
        let j = i16::from_le_bytes([payload[11], payload[12]]);
        let k = i16::from_le_bytes([payload[13], payload[14]]);
        let w = i16::from_le_bytes([payload[15], payload[16]]);
        println!("    quat raw: i={} j={} k={} w={}", i, j, k, w);
        println!("    quat f64: i={:.4} j={:.4} k={:.4} w={:.4}",
                 i as f64 / 16384.0, j as f64 / 16384.0,
                 k as f64 / 16384.0, w as f64 / 16384.0);
    }
    if payload.len() >= 15 && report_id == 0x02 {
        let x = i16::from_le_bytes([payload[9],  payload[10]]);
        let y = i16::from_le_bytes([payload[11], payload[12]]);
        let z = i16::from_le_bytes([payload[13], payload[14]]);
        println!("    gyro raw: x={} y={} z={}", x, y, z);
        println!("    gyro rad/s: x={:.4} y={:.4} z={:.4}",
                 x as f64 / 512.0, y as f64 / 512.0, z as f64 / 512.0);
    }
}

fn drain_print(dev: &mut I2cdev, label: &str, duration_ms: u64) {
    println!("\n=== {} ({} ms) ===", label, duration_ms);
    let deadline = Instant::now() + Duration::from_millis(duration_ms);
    let mut count = 0;
    while Instant::now() < deadline {
        match try_read_packet(dev) {
            None => {
                thread::sleep(Duration::from_millis(2));
            }
            Some((ch, payload)) => {
                count += 1;
                println!("  Packet #{}: ch={} ({}) payload_len={}",
                         count, ch, channel_name(ch), payload.len());
                if !payload.is_empty() {
                    println!("  payload: {}", hex(&payload));
                }
                if ch == CH_REPORTS && !payload.is_empty() {
                    annotate_sensor_report(&payload);
                }
                println!();
            }
        }
    }
    println!("  → {} packets in {} ms", count, duration_ms);
}

fn main() -> Result<()> {
    println!("BNO08X SHTP Raw Packet Debugger");
    println!("================================");
    println!("Device: {}  Address: 0x{:02X}\n", I2C_DEV, ADDR);

    let mut dev = I2cdev::new(I2C_DEV)?;
    let mut seq = [0u8; 8];

    // --- Phase 1: read boot packets before any reset ---
    drain_print(&mut dev, "PHASE 1: Boot packets (no reset)", 500);

    // --- Phase 2: soft reset ---
    println!("\n=== PHASE 2: Soft reset ===");
    write_packet(&mut dev, &mut seq, CH_EXECUTABLE, &[0x01])?;
    println!("  Sleeping 1000 ms...");
    thread::sleep(Duration::from_millis(1000));
    drain_print(&mut dev, "Post-reset drain", 500);

    // --- Phase 3: enable Game Rotation Vector (0x08) at 10 ms ---
    println!("\n=== PHASE 3: Enable Game Rotation Vector (0x08) @ 10ms ===");
    let period_us: u32 = 10_000;
    let p = period_us.to_le_bytes();
    write_packet(&mut dev, &mut seq, CH_CONTROL, &[
        0xFD, 0x08,           // SET_FEATURE_COMMAND, Game Rotation Vector
        0, 0, 0,              // flags, sensitivity
        p[0], p[1], p[2], p[3], // report interval
        0, 0, 0, 0,           // batch interval
        0, 0, 0, 0,           // sensor-specific config
    ])?;
    thread::sleep(Duration::from_millis(50));

    // --- Phase 4: enable Gyro Calibrated (0x02) at 10 ms ---
    println!("\n=== PHASE 4: Enable Gyro Calibrated (0x02) @ 10ms ===");
    write_packet(&mut dev, &mut seq, CH_CONTROL, &[
        0xFD, 0x02,
        0, 0, 0,
        p[0], p[1], p[2], p[3],
        0, 0, 0, 0,
        0, 0, 0, 0,
    ])?;
    thread::sleep(Duration::from_millis(50));

    // --- Phase 5: read sensor reports for 3 seconds ---
    drain_print(&mut dev, "PHASE 5: Sensor reports (3 s)", 3000);

    Ok(())
}
