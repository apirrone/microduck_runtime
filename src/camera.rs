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
        "labels_file":  "/usr/share/imx500-models/coco_labels.txt",
        "confidence_threshold": 0.4,
        "iou_threshold":        0.3,
        "max_detections":       10
    }
}
"#;

const PP_CONFIG_PATH: &str = "/tmp/microduck_imx500_pp.json";

// ── COCO 80-class labels (SSD MobileNetV2 sorted order, 0-indexed) ────────────
const COCO_LABELS: [&str; 80] = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
];

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
    pub fn start(width: u32, height: u32, fps: u32, ball_detect: bool) -> Self {
        let latest_frame: FrameBuffer          = Arc::new(Mutex::new(None));
        let latest_detections: DetectionBuffer = Arc::new(Mutex::new(None));

        let frame_buf = Arc::clone(&latest_frame);
        let det_buf   = Arc::clone(&latest_detections);

        thread::spawn(move || {
            loop {
                if let Err(e) = capture_loop(
                    width, height, fps, ball_detect,
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

    if ball_detect {
        // Write the IMX500 post-process config to a temp file (once per restart)
        write_pp_config()?;
        cmd.args(["--post-process-file", PP_CONFIG_PATH,
                  "--verbose", "1"]);
        eprintln!("[camera] ball-detect mode: IMX500 SSD inference enabled");
    }

    let mut child: Child = cmd
        .stdout(Stdio::piped())
        .stderr(if ball_detect { Stdio::piped() } else { Stdio::null() })
        .spawn()
        .with_context(|| format!("Failed to spawn {binary}"))?;

    // In ball-detect mode: parse detection lines from stderr in a side thread.
    if ball_detect {
        if let Some(stderr) = child.stderr.take() {
            let det_buf = Arc::clone(&det);
            thread::spawn(move || {
                let reader = std::io::BufReader::new(stderr);
                let mut pending: Vec<String> = Vec::new();

                for line in reader.lines().flatten() {
                    // rpicam-apps prints "imx500_object_detection:" then one
                    // line per object.  We buffer lines until we see the start
                    // of the next block, then flush accumulated detections.
                    if line.trim_start().starts_with("imx500_object_detection:") {
                        // Flush previous batch
                        if !pending.is_empty() {
                            let json = format!("[{}]", pending.join(","));
                            let mut lock = det_buf.lock().unwrap();
                            *lock = Some(json);
                            pending.clear();
                        }
                    } else if let Some(det_json) = parse_detection_line(&line) {
                        pending.push(det_json);
                    } else if !line.trim().is_empty() {
                        // Pass through other stderr messages so they appear in journalctl
                        eprintln!("[rpicam] {line}");
                    }
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
// rpicam-apps imx500_object_detection prints detected objects to stderr when
// --verbose 1 is set.  The format varies slightly by version; we handle the
// most common patterns below.
//
// Known formats:
//   "  sports ball (0.870) : 100 200 50 50"           (pixel bbox)
//   "  sports ball 0.870 : 100 200 50 50"             (no parens)
//   "  sports ball 0.870 100 200 50 50"               (no colon)
//   "[INFO] …  sports ball (0.870) : 100 200 50 50"   (with log prefix)
//   "  sports ball (0.870) : [0.10, 0.20, 0.30, 0.40]" (normalised bbox)
//
// Returns a JSON object string like:
//   {"class":"sports ball","score":0.87,"box":[100,200,50,50]}

fn parse_detection_line(line: &str) -> Option<String> {
    // Strip common log prefixes: timestamps, "[INFO]", "INFO : ", etc.
    let s = strip_log_prefix(line.trim());
    if s.is_empty() { return None; }

    // We expect at least a label + a number somewhere in the line.
    // Use a simple state machine: scan forward for a label (non-numeric
    // prefix tokens), then a confidence float, then 4 bbox numbers.

    // Split on common delimiters
    let tokens: Vec<&str> = s
        .split(|c: char| c == '(' || c == ')' || c == ':' || c == '[' || c == ']' || c == ',')
        .flat_map(|t| t.split_whitespace())
        .collect();

    if tokens.len() < 6 {
        return None;
    }

    // Find first float that looks like a confidence score (0 < v ≤ 1)
    let conf_pos = tokens.iter().position(|t| {
        t.parse::<f32>().map(|v| (0.0..=1.0).contains(&v)).unwrap_or(false)
    })?;

    if conf_pos == 0 {
        return None; // no label prefix
    }

    let label: String = tokens[..conf_pos].join(" ");
    let score: f32 = tokens[conf_pos].parse().ok()?;

    // Collect up to 4 numbers after the confidence for the bounding box
    let mut bbox: Vec<f32> = Vec::new();
    for tok in &tokens[conf_pos + 1..] {
        if let Ok(v) = tok.parse::<f32>() {
            bbox.push(v);
            if bbox.len() == 4 { break; }
        }
    }
    if bbox.len() < 4 {
        return None;
    }

    let [x, y, w, h] = [bbox[0], bbox[1], bbox[2], bbox[3]];

    // Validate the label is a known COCO class (or at least non-empty)
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
