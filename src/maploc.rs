//! Glue between the runtime and the `microduck_maploc` crate.
//!
//! Holds the occupancy grid, MCL localizer, waypoint follower, and the
//! optional telemetry/goal sockets. The runtime calls `tick` every
//! control loop with the latest odometry pose + an optional ToF scan,
//! and `velocity_cmd` to ask "should I be auto-driving along a path?".
//!
//! Maploc is autonomous on its own (`--maploc`); the streaming sidecar
//! (`--stream` paired with `--maploc`) just exposes telemetry and accepts
//! goal clicks from a laptop viewer.

use std::path::PathBuf;
use std::time::Instant;

use anyhow::{Context, Result};

use microduck_maploc::{
    follower::{follow_step, FollowCommand, FollowerState},
    grid::{GridConfig, OccupancyGrid},
    mcl::{Localizer, MclConfig},
    planner::{plan, PlannerConfig},
    stream::{GoalServer, Telemetry},
    wire,
};

use crate::tof::{TofFrame, TofSource};

/// Maploc hyperparameters surfaced to the runtime CLI.
#[derive(Debug, Clone)]
pub struct MaplocOptions {
    pub map_path:      PathBuf,
    pub wipe:          bool,
    pub stream:        bool,
    pub stream_port:   u16,
    pub goal_port:     u16,
    /// Linear command in m/s when the follower says "go forward".
    pub lin_speed:     f32,
    /// Yaw rate command in rad/s when turning.
    pub yaw_speed:     f32,
    pub arrive_radius: f32,
    /// Send the full map blob this often (seconds). Higher = less wifi load.
    pub map_send_period_s: f32,
}

impl Default for MaplocOptions {
    fn default() -> Self {
        Self {
            map_path:      PathBuf::from("/var/lib/microduck/maploc_map.bin"),
            wipe:          false,
            stream:        false,
            stream_port:   9874,
            goal_port:     9875,
            lin_speed:     0.20,
            yaw_speed:     1.20,
            arrive_radius: 0.10,
            map_send_period_s: 1.0,
        }
    }
}

pub struct Maploc {
    grid:      OccupancyGrid,
    localizer: Localizer,
    follower:  FollowerState,
    telemetry: Option<Telemetry>,
    goals:     Option<GoalServer>,
    /// Last (x, y, yaw) we passed to the localizer's `predict`. Used to
    /// compute body-frame deltas from successive odometry readings.
    last_odom: Option<(f32, f32, f32)>,
    last_map_send: Instant,
    started_at:    Instant,
    opts:          MaplocOptions,
    /// Cached "we just arrived" flag — flipped to true the moment the
    /// follower drains its waypoints, so the caller can clear the path
    /// on the wire and stop driving.
    just_finished: bool,
}

impl Maploc {
    pub fn new(opts: MaplocOptions) -> Result<Self> {
        let grid = if opts.wipe {
            OccupancyGrid::new(GridConfig::default())
        } else {
            match OccupancyGrid::load(&opts.map_path)
                .context("loading saved map")?
            {
                Some(g) => {
                    eprintln!("[maploc] loaded map from {}", opts.map_path.display());
                    g
                }
                None => {
                    eprintln!("[maploc] no saved map at {}; starting fresh",
                              opts.map_path.display());
                    OccupancyGrid::new(GridConfig::default())
                }
            }
        };
        let localizer = Localizer::new(&grid, MclConfig::default(), 0);

        let (telemetry, goals) = if opts.stream {
            let tel = Telemetry::bind(opts.stream_port)
                .with_context(|| format!("bind telemetry on {}", opts.stream_port))?;
            let g = GoalServer::bind(opts.goal_port)
                .with_context(|| format!("bind goal server on {}", opts.goal_port))?;
            eprintln!("[maploc] streaming on telemetry={} goal={}",
                      opts.stream_port, opts.goal_port);
            (Some(tel), Some(g))
        } else {
            (None, None)
        };

        Ok(Self {
            grid, localizer,
            follower: FollowerState::empty(),
            telemetry, goals,
            last_odom: None,
            last_map_send: Instant::now(),
            started_at:    Instant::now(),
            opts,
            just_finished: false,
        })
    }

    /// Initialize with a known starting pose (e.g. on the dock).
    pub fn reset_known(&mut self, x: f32, y: f32, yaw: f32) {
        self.localizer.reset_known(x, y, yaw);
        self.last_odom = Some((x, y, yaw));
    }

    /// Force re-uniform — the kidnapped-robot state. Drops any active path.
    pub fn reset_uniform(&mut self) {
        self.localizer.reset_uniform(&self.grid);
        self.follower = FollowerState::empty();
        if let Some(tel) = &mut self.telemetry {
            tel.send_path(&wire::Path { waypoints: vec![] });
        }
    }

    /// One main-loop tick. Returns the body-frame velocity command the
    /// runtime should pipe into the locomotion controller (zero when no
    /// path is active).
    ///
    /// `odom_xy_yaw` is the latest dead-reckoned pose from the runtime's
    /// `odometry::Odometry` engine — we use successive deltas as the
    /// motion model input.
    /// `tof` is consulted for a fresh frame; pass `&mut NoopTof` if no
    /// hardware is connected yet.
    /// `dt` is the seconds since the last `tick` call.
    pub fn tick(
        &mut self,
        odom_xy_yaw: (f32, f32, f32),
        tof: &mut dyn TofSource,
        dt: f32,
    ) -> FollowCommand {
        let (ox, oy, oyaw) = odom_xy_yaw;
        // Motion model — body-frame delta from previous odom checkpoint.
        if let Some((px, py, pyaw)) = self.last_odom {
            let dx_w = ox - px;
            let dy_w = oy - py;
            let dyaw = wrap_pi(oyaw - pyaw);
            let cp = pyaw.cos(); let sp = pyaw.sin();
            let dx_b =  cp * dx_w + sp * dy_w;
            let dy_b = -sp * dx_w + cp * dy_w;
            self.localizer.predict(dx_b, dy_b, dyaw);
        }
        self.last_odom = Some((ox, oy, oyaw));

        // Measurement update + map ingestion.
        let now_s = self.started_at.elapsed().as_secs_f64();
        if let Some(frame) = tof.next_frame(now_s) {
            self.localizer.update(&self.grid, &frame.angles_body, &frame.ranges);
            self.integrate_scan_into_map(&frame);
            // Tagged for telemetry.
            if let Some(tel) = &mut self.telemetry {
                tel.send_scan(&wire::Scan {
                    angles_body: frame.angles_body.clone(),
                    ranges:      frame.ranges.clone(),
                    origin:      frame.origin,
                });
            }
        }

        // Drain incoming goals → plan a path → stash in follower.
        if let Some(srv) = &mut self.goals {
            if let Some(goal) = srv.tick() {
                self.handle_goal(goal);
            }
        }

        // Step the follower against the *estimated* pose.
        let (ex, ey, eyaw) = self.localizer.best();
        let was_done = self.follower.done();
        let cmd = follow_step(
            &mut self.follower,
            (ex, ey), eyaw,
            self.opts.lin_speed, self.opts.yaw_speed, dt,
            self.opts.arrive_radius,
        );
        if !was_done && self.follower.done() {
            self.just_finished = true;
            if let Some(tel) = &mut self.telemetry {
                tel.send_path(&wire::Path { waypoints: vec![] });
            }
        }

        // Stream pose (always, when telemetry is up).
        if let Some(tel) = &mut self.telemetry {
            tel.poll_accept();
            let lock = if self.localizer.position_std() < 0.20
                && self.localizer.last_residual_m().is_finite()
                && self.localizer.last_residual_m() < 0.30
            {
                wire::LockState::Tracking
            } else {
                wire::LockState::Searching
            };
            tel.send_pose(wire::Pose {
                x: ex, y: ey, yaw: eyaw,
                std_xy: self.localizer.position_std(),
                residual_m: if self.localizer.last_residual_m().is_finite() {
                    self.localizer.last_residual_m()
                } else { 0.0 },
                lock,
                timestamp_ms: self.started_at.elapsed().as_millis() as u64,
            });
            // Periodic map blob.
            if self.last_map_send.elapsed().as_secs_f32() >= self.opts.map_send_period_s {
                if let Ok(blob) = encode_map_blob(&self.grid) {
                    tel.send_map(&blob);
                }
                self.last_map_send = Instant::now();
            }
        }

        cmd
    }

    fn handle_goal(&mut self, goal: wire::Goal) {
        let (ex, ey, _) = self.localizer.best();
        match plan(&self.grid, (ex, ey), (goal.x, goal.y), PlannerConfig::default()) {
            Some(p) if p.len() >= 2 => {
                // Drop the first waypoint — it's where we are.
                let waypoints: Vec<(f32, f32)> = p[1..].to_vec();
                eprintln!("[maploc] goal=({:.2},{:.2}) → {} waypoints",
                          goal.x, goal.y, waypoints.len());
                if let Some(tel) = &mut self.telemetry {
                    tel.send_path(&wire::Path { waypoints: p.clone() });
                }
                self.follower = FollowerState::new(waypoints);
            }
            _ => {
                eprintln!("[maploc] no path to goal=({:.2},{:.2})", goal.x, goal.y);
                self.follower = FollowerState::empty();
                if let Some(tel) = &mut self.telemetry {
                    tel.send_path(&wire::Path { waypoints: vec![] });
                }
            }
        }
    }

    fn integrate_scan_into_map(&mut self, frame: &TofFrame) {
        let (ox, oy) = frame.origin;
        let (_, _, eyaw) = self.localizer.best();
        for (a, &r) in frame.angles_body.iter().zip(&frame.ranges) {
            if !r.is_finite() || r <= 0.0 { continue; }
            let theta = eyaw + a;
            let hx = ox + r * theta.cos();
            let hy = oy + r * theta.sin();
            // We've already projected to the horizontal plane upstream,
            // so every finite return is a wall hit.
            self.grid.integrate_ray(ox, oy, hx, hy, true);
        }
    }

    pub fn save_map(&self) -> Result<()> {
        self.grid.save(&self.opts.map_path)
            .with_context(|| format!("saving map to {}", self.opts.map_path.display()))
    }

    /// True for one tick after the duck arrives at the last waypoint —
    /// the runtime can use this to e.g. emit a "destination reached"
    /// chirp or post to the brain.
    pub fn just_finished(&mut self) -> bool {
        let v = self.just_finished;
        self.just_finished = false;
        v
    }

    pub fn estimate(&self) -> (f32, f32, f32) { self.localizer.best() }
    pub fn position_std(&self) -> f32         { self.localizer.position_std() }
    pub fn has_active_path(&self) -> bool     { !self.follower.done() }
}

fn wrap_pi(a: f32) -> f32 {
    let mut x = a;
    while x >  std::f32::consts::PI { x -= 2.0 * std::f32::consts::PI; }
    while x < -std::f32::consts::PI { x += 2.0 * std::f32::consts::PI; }
    x
}

/// Save the grid to an in-memory buffer matching the on-wire / on-disk format.
fn encode_map_blob(grid: &OccupancyGrid) -> std::io::Result<Vec<u8>> {
    // Round-trip via a temp file is silly; instead use a Cursor-backed
    // writer the public API doesn't expose, so we reach for save() to a
    // tempfile and read it back. Cheap (~26 KB once a second).
    let tmp = std::env::temp_dir().join(format!("microduck_maploc_send_{}.bin",
        std::process::id()));
    grid.save(&tmp)?;
    let bytes = std::fs::read(&tmp)?;
    let _ = std::fs::remove_file(&tmp);
    Ok(bytes)
}
