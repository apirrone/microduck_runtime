//! Background camera capture via `rpicam-vid`/`libcamera-vid`.
//!
//! Normal mode: streams MJPEG bytes from the child's stdout, parses JPEG
//! frame boundaries (SOI=FFD8 … EOI=FFD9) and publishes each complete frame
//! into `latest_frame`.
//!
//! Ball-detect mode: additionally passes `--post-process-file` pointing to a
//! temporary IMX500 SSD object-detection config.  rpicam-vid runs the neural
//! network on-sensor and prints detected objects to stderr; a second thread
//! parses those lines and publishes a JSON array into `latest_detections`.
//!
//! ## Detection output format
//!
//! rpicam-apps (imx500_object_detection stage) prints one line per detected
//! object to stderr when --verbose 1 is set.  The most common format is:
//!
//!   `  sports ball (0.870) : 100 200 50 50`
//!
//! where the four numbers are pixel-space x, y, w, h.  The parser in this
//! file handles the formats seen across rpicam-apps versions; adjust
//! `parse_detection_line()` if the format on your Pi differs.

use anyhow::{Context, Result};
use std::io::{BufRead, Read, Write as IoWrite};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// Shared latest JPEG frame.  `None` until the first frame arrives or after
/// the frame has been consumed by the sender.
pub type FrameBuffer = Arc<Mutex<Option<Vec<u8>>>>;

/// Shared latest detection JSON string (a JSON array, e.g.
/// `[{"class":"sports ball","score":0.9,"box":[x,y,w,h]}]`).
/// Always `None` when ball-detect mode is off.
pub type DetectionBuffer = Arc<Mutex<Option<String>>>;

// ── IMX500 post-process config ────────────────────────────────────────────────
//
// Written to a temp file and passed via --post-process-file.
// Instructs rpicam-apps to load the SSD MobileNetV2 COCO model onto the
// IMX500 and run inference every frame.  Requires:
//   sudo apt install imx500-all    (installs .rpk models + rpicam-apps stage)
const IMX500_PP_CONFIG: &str = r#"{
    "imx500_object_detection": {
        "network_file": "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk",
        "threshold":     0.3,
        "max_detections": 10,
        "temporal_filter": {
            "tolerance": 0.1,
            "factor": 0.2,
            "visible_frames": 4,
            "hidden_frames": 2
        },
        "classes": [
            "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
            "truck", "boat", "traffic light", "fire hydrant", "-", "stop sign",
            "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
            "elephant", "bear", "zebra", "giraffe", "-", "backpack", "umbrella",
            "-", "-", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
            "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
            "surfboard", "tennis racket", "bottle", "-", "wine glass", "cup", "fork",
            "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
            "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
            "couch", "potted plant", "bed", "-", "dining table", "-", "-", "toilet",
            "-", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "-", "book",
            "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        ]
    }
}
"#;

const PP_CONFIG_PATH: &str = "/tmp/microduck_imx500_pp.json";

// ── Public API ────────────────────────────────────────────────────────────────

/// Manages the camera subprocess and publishes JPEG frames (and optionally
/// IMX500 detection results).
pub struct CameraStream {
    /// Take `Some(jpeg_bytes)` out of here to consume the latest frame.
    pub latest_frame: FrameBuffer,
    /// Take `Some(json_string)` out of here to consume the latest detections.
    /// Always `None` when ball-detect mode is off.
    pub latest_detections: DetectionBuffer,
}

impl CameraStream {
    /// Spawn the capture thread(s) and return immediately.
    ///
    /// * `ball_detect` — when `true`, pass `--post-process-file` to rpicam-vid
    ///   so the IMX500 runs onboard inference; parse detection lines from stderr.
    pub fn start(width: u32, height: u32, fps: u32,
                 shutter_us: u32, gain: f32,
                 ball_detect: bool) -> Self {
        let latest_frame: FrameBuffer          = Arc::new(Mutex::new(None));
        let latest_detections: DetectionBuffer = Arc::new(Mutex::new(None));

        let frame_buf = Arc::clone(&latest_frame);
        let det_buf   = Arc::clone(&latest_detections);

        thread::spawn(move || {
            loop {
                if let Err(e) = capture_loop(
                    width, height, fps, shutter_us, gain, ball_detect,
                    Arc::clone(&frame_buf),
                    Arc::clone(&det_buf),
                ) {
                    eprintln!("[camera] capture error: {e}");
                }
                eprintln!("[camera] restarting in 2 s …");
                thread::sleep(Duration::from_secs(2));
            }
        });

        Self { latest_frame, latest_detections }
    }
}

// ── Capture loop ──────────────────────────────────────────────────────────────

fn capture_loop(
    width: u32, height: u32, fps: u32,
    shutter_us: u32, gain: f32,
    ball_detect: bool,
    buf: FrameBuffer,
    det: DetectionBuffer,
) -> Result<()> {
    // On Pi OS Bookworm the binary is rpicam-vid; libcamera-vid is a symlink.
    let binary = if Command::new("which")
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

    let mut cmd = Command::new(binary);
    cmd.args(["--codec", "mjpeg", "--output", "-",
              "--width", &w, "--height", &h, "--framerate", &f,
              "--nopreview", "--timeout", "0"]);

    // Manual shutter: disables AGC and caps exposure. Pair with gain so
    // frames stay reasonably bright — very low shutter needs good light
    // or a high gain (at the cost of noise).
    let shutter_str;
    let gain_str;
    if shutter_us > 0 {
        shutter_str = shutter_us.to_string();
        gain_str = format!("{:.1}", gain);
        cmd.args(["--shutter", &shutter_str, "--gain", &gain_str,
                  "--awb", "auto"]);
        eprintln!("[camera] manual shutter: {} µs, gain {}", shutter_us, gain_str);
    }

    if ball_detect {
        // Write the IMX500 post-process config to a temp file (once per restart)
        write_pp_config()?;
        cmd.args(["--post-process-file", PP_CONFIG_PATH,
                  "--verbose", "2"]);
        eprintln!("[camera] ball-detect mode: IMX500 SSD inference enabled");
    }

    let mut child: Child = cmd
        .stdout(Stdio::piped())
        .stderr(if ball_detect { Stdio::piped() } else { Stdio::null() })
        .spawn()
        .with_context(|| format!("Failed to spawn {binary}"))?;

    // In ball-detect mode: two threads — one blocks on stderr lines, one
    // flushes the shared pending list every 200 ms.  Splitting them means
    // the timer keeps firing (sending `[]` heartbeats) even when rpicam-apps
    // goes quiet after the model loads, which keeps the TCP client connected.
    if ball_detect {
        if let Some(stderr) = child.stderr.take() {
            // Shared list of parsed detection JSON fragments not yet flushed.
            let pending: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));

            // Thread 1: blocking stderr reader — appends parsed detections.
            let pending_reader = Arc::clone(&pending);
            thread::spawn(move || {
                let reader = std::io::BufReader::new(stderr);
                for line in reader.lines().flatten() {
                    if let Some(det_json) = parse_detection_line(&line) {
                        eprintln!("[detect] {}", line.trim());
                        pending_reader.lock().unwrap().push(det_json);
                    }
                }
            });

            // Thread 2: timer — drains pending into det_buf every 200 ms.
            // Sends `[]` when nothing was detected, keeping the TCP stream alive.
            let det_buf_timer = Arc::clone(&det);
            thread::spawn(move || {
                loop {
                    thread::sleep(Duration::from_millis(200));
                    let mut p = pending.lock().unwrap();
                    let json = format!("[{}]", p.join(","));
                    p.clear();
                    drop(p);
                    let mut lock = det_buf_timer.lock().unwrap();
                    *lock = Some(json);
                }
            });
        }
    }

    // Read JPEG frames from stdout (same for both modes).
    let mut stdout = child.stdout.take()
        .ok_or_else(|| anyhow::anyhow!("rpicam-vid produced no stdout"))?;

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
                if soi > 0 { ring.drain(..soi); }
                break;
            };
            let end = soi + 2 + rel + 2;
            {
                let mut lock = buf.lock().unwrap();
                *lock = Some(ring[soi..end].to_vec());
            }
            ring.drain(..end);
        }

        if ring.len() > 1024 * 1024 {
            eprintln!("[camera] ring buffer overflow — clearing");
            ring.clear();
        }
    }

    child.wait()?;
    Ok(())
}

// ── Detection line parser ─────────────────────────────────────────────────────
//
// rpicam-apps imx500_object_detection prints one line per detected object:
//
//   "[0] : sports ball[36] (0.38) @ 179,248 97x89"
//
// strip_log_prefix() consumes the "[0]" index, leaving:
//
//   ": sports ball[36] (0.38) @ 179,248 97x89"
//
// Returns a JSON object string like:
//   {"class":"sports ball","score":0.380,"box":[179.0,248.0,97.0,89.0]}

fn parse_detection_line(line: &str) -> Option<String> {
    let s = strip_log_prefix(line.trim());

    // After stripping the "[N]" index prefix, detection lines start with ": "
    let rest = s.strip_prefix(": ")?.trim();
    // rest = "sports ball[36] (0.38) @ 179,248 97x89"

    // Split on " @ " to separate "label[id] (score)" from "x,y wxh"
    let (label_score, coords_str) = rest.split_once(" @ ")?;

    // Extract label: everything before the "[class_id]" bracket
    let bracket = label_score.rfind('[')?;
    let label = label_score[..bracket].trim();

    // Extract score from "(0.38)"
    let p0 = label_score.find('(')?;
    let p1 = label_score.find(')')?;
    let score: f32 = label_score[p0 + 1..p1].trim().parse().ok()?;

    // Parse "179,248 97x89"
    let (xy_str, wh_str) = coords_str.trim().split_once(' ')?;
    let (xs, ys) = xy_str.split_once(',')?;
    let (ws, hs) = wh_str.split_once('x')?;
    let x: f32 = xs.trim().parse().ok()?;
    let y: f32 = ys.trim().parse().ok()?;
    let w: f32 = ws.trim().parse().ok()?;
    let h: f32 = hs.trim().parse().ok()?;

    if label.is_empty() { return None; }

    Some(format!(
        r#"{{"class":"{label}","score":{score:.3},"box":[{x:.1},{y:.1},{w:.1},{h:.1}]}}"#
    ))
}

/// Strip leading log prefixes like "[2024-01-01 12:00:00.000] [INFO] [file:42] "
/// or "INFO : ".
fn strip_log_prefix(s: &str) -> &str {
    // Strip bracketed timestamp/level blocks
    let mut rest = s;
    while rest.starts_with('[') {
        if let Some(end) = rest.find(']') {
            rest = rest[end + 1..].trim_start();
        } else {
            break;
        }
    }
    // Strip "INFO : " or "INFO: "
    if rest.starts_with("INFO") {
        if let Some(pos) = rest.find(':') {
            rest = rest[pos + 1..].trim_start();
        }
    }
    rest
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Write the IMX500 post-process config to the temp file.
fn write_pp_config() -> Result<()> {
    let mut f = std::fs::File::create(PP_CONFIG_PATH)
        .with_context(|| format!("Failed to create {PP_CONFIG_PATH}"))?;
    f.write_all(IMX500_PP_CONFIG.as_bytes())
        .with_context(|| format!("Failed to write {PP_CONFIG_PATH}"))?;
    Ok(())
}

/// Find the first occurrence of the two-byte sequence `[a, b]` in `data`.
fn find2(data: &[u8], a: u8, b: u8) -> Option<usize> {
    data.windows(2).position(|w| w[0] == a && w[1] == b)
}
