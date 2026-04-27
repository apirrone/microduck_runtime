//! ToF sensor abstraction for `microduck_maploc`.
//!
//! Defines the data shape the maploc layer expects and a `TofSource`
//! trait so the actual VL53L5CX driver (TBD) can plug in alongside
//! cheap stub/recorded sources used during development.
//!
//! Convention: angles are in the body frame (+x forward, +y left, +z
//! up); ranges are projected into the horizontal plane already (a
//! beam looking +5° up at distance r contributes `r * cos(5°)` here).
//! The `microduck_maploc` crate consumes those directly.

/// One scan's worth of horizontal-plane ranges.
#[derive(Debug, Clone, Default)]
pub struct TofFrame {
    pub angles_body: Vec<f32>,   // body-frame azimuths, radians
    pub ranges:      Vec<f32>,   // metres; NaN = no return
    pub origin:      (f32, f32), // sensor origin (world XY) at the time of capture
    pub timestamp_s: f64,        // monotonic seconds
}

pub trait TofSource: Send {
    /// Return the next available frame, or `None` if no fresh data is
    /// ready yet. Non-blocking; the runtime calls this every tick and
    /// passes the result (whether `Some` or `None`) into `Maploc::tick`.
    fn next_frame(&mut self, now_s: f64) -> Option<TofFrame>;
}

/// Stub source — produces nothing. Use until the real VL53L5CX driver
/// is wired in. Maploc will keep predicting on odometry but never
/// update with measurements; the duck's pose estimate drifts at the
/// odometry rate, which is fine for compile-and-smoke-test.
pub struct NoopTof;

impl TofSource for NoopTof {
    fn next_frame(&mut self, _now_s: f64) -> Option<TofFrame> { None }
}
