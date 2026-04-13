//! Background camera capture via `libcamera-vid --codec mjpeg`.
//!
//! Spawns a child process that streams raw MJPEG bytes to stdout.
//! A background thread parses JPEG frame boundaries (SOI=FFD8 … EOI=FFD9)
//! and publishes each complete frame into a shared buffer.
//! The main thread takes frames out of the buffer whenever it is ready to send.

use anyhow::{Context, Result};
use std::io::Read;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Shared latest JPEG frame.  `None` until the first frame arrives or after
/// the frame has been consumed by the sender.
pub type FrameBuffer = Arc<Mutex<Option<Vec<u8>>>>;

/// Manages the libcamera-vid subprocess and publishes JPEG frames.
pub struct CameraStream {
    /// Take `Some(jpeg_bytes)` out of here to consume the latest frame.
    pub latest_frame: FrameBuffer,
}

impl CameraStream {
    /// Spawn the capture thread and return immediately.
    pub fn start(width: u32, height: u32, fps: u32) -> Self {
        let latest_frame: FrameBuffer = Arc::new(Mutex::new(None));
        let buf = Arc::clone(&latest_frame);

        thread::spawn(move || {
            loop {
                if let Err(e) = capture_loop(width, height, fps, Arc::clone(&buf)) {
                    eprintln!("[camera] capture error: {e}");
                }
                eprintln!("[camera] restarting in 2 s …");
                thread::sleep(Duration::from_secs(2));
            }
        });

        Self { latest_frame }
    }
}

// ── capture loop ─────────────────────────────────────────────────────────────

fn capture_loop(width: u32, height: u32, fps: u32, buf: FrameBuffer) -> Result<()> {
    // On Pi OS Bookworm the binary is rpicam-vid; libcamera-vid is a compat symlink.
    // Try rpicam-vid first, fall back to libcamera-vid.
    let binary = if std::process::Command::new("which")
        .arg("rpicam-vid").output()
        .map(|o| o.status.success()).unwrap_or(false)
    {
        "rpicam-vid"
    } else {
        "libcamera-vid"
    };

    let w = width.to_string();
    let h = height.to_string();
    let f = fps.to_string();

    let mut child: Child = Command::new(binary)
        .args(["--codec", "mjpeg", "--output", "-",
               "--width", &w, "--height", &h, "--framerate", &f,
               "--nopreview", "--timeout", "0"])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .with_context(|| format!("Failed to spawn {binary}"))?;

    let mut stdout = child.stdout.take()
        .ok_or_else(|| anyhow::anyhow!("libcamera-vid produced no stdout"))?;

    // Accumulation buffer for incomplete frame data
    let mut ring: Vec<u8> = Vec::with_capacity(128 * 1024);
    let mut tmp  = [0u8; 8192];

    loop {
        let n = stdout.read(&mut tmp)?;
        if n == 0 {
            break; // process exited
        }
        ring.extend_from_slice(&tmp[..n]);

        // Extract every complete JPEG: SOI (FF D8) … EOI (FF D9, inclusive)
        loop {
            let Some(soi) = find2(&ring, 0xFF, 0xD8) else { break };
            let Some(rel) = find2(&ring[soi + 2..], 0xFF, 0xD9) else {
                // EOI not yet arrived — trim leading garbage before SOI
                if soi > 0 { ring.drain(..soi); }
                break;
            };
            let end = soi + 2 + rel + 2; // byte position after FF D9
            {
                let mut lock = buf.lock().unwrap();
                *lock = Some(ring[soi..end].to_vec());
            }
            ring.drain(..end);
        }

        // Guard against unbounded growth if libcamera sends garbage
        if ring.len() > 1024 * 1024 {
            eprintln!("[camera] ring buffer overflow — clearing");
            ring.clear();
        }
    }

    child.wait()?;
    Ok(())
}

// ── helpers ───────────────────────────────────────────────────────────────────

/// Find the first occurrence of the two-byte sequence `[a, b]` in `data`.
fn find2(data: &[u8], a: u8, b: u8) -> Option<usize> {
    data.windows(2).position(|w| w[0] == a && w[1] == b)
}
