//! Contact-based odometry for microduck, computed inline in Rust.
//!
//! FK chains are hardcoded directly from the MJCF (robot_standup.xml) —
//! single source of truth, no external file required at runtime.
//!
//! Algorithm (ported from Rhoban humanoid model_service.cpp):
//!   - 4 candidate contact corners per foot sole
//!   - Each step: find the corner with the lowest world Z
//!   - If lower than the current anchor (which sits at z≈0), switch to it
//!   - Re-project trunk: anchor at flat ground (z=0), oriented by IMU quat

use std::collections::HashMap;
use nalgebra::{Isometry3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

// ---------------------------------------------------------------------------
// Motor → joint mapping (runtime motor vector order)
// ---------------------------------------------------------------------------

/// Motor index → joint name.  Index 9 (mouth) has no kinematic joint.
const MOTOR_JOINTS: [Option<&str>; 15] = [
    Some("left_hip_yaw"),    // 0
    Some("left_hip_roll"),   // 1
    Some("left_hip_pitch"),  // 2
    Some("left_knee"),       // 3
    Some("left_ankle"),      // 4
    Some("neck_pitch"),      // 5
    Some("head_pitch"),      // 6
    Some("head_yaw"),        // 7
    Some("head_roll"),       // 8
    None,                    // 9 — mouth, no kinematic effect
    Some("right_hip_yaw"),   // 10
    Some("right_hip_roll"),  // 11
    Some("right_hip_pitch"), // 12
    Some("right_knee"),      // 13
    Some("right_ankle"),     // 14
];

// ---------------------------------------------------------------------------
// Contact geometry
// ---------------------------------------------------------------------------

/// Half-length of the foot sole along the foot-site frame X axis (front/back).
/// Derived from foot_tpu_bottom.stl: sole is 51 mm long → half = 25.5 mm.
const FOOT_SOLE_HALF_LEN: f64 = 0.0255;

/// Half-width of the foot sole along the foot-site frame Y axis (left/right).
/// Derived from foot_tpu_bottom.stl: sole is ~40 mm wide → half = 20 mm.
const FOOT_SOLE_HALF_WIDTH: f64 = 0.020;

/// World-Z threshold for contact switching (see SWITCH_CONFIRM_TICKS).
const SWITCH_MARGIN: f64 = -0.010;

/// Ticks a candidate must hold the lowest-point position before the anchor
/// switches.  At 50 Hz this is 40 ms — well within a normal step duration.
const SWITCH_CONFIRM_TICKS: u32 = 2;

// ---------------------------------------------------------------------------
// MJCF-based kinematic chain (robot_standup.xml — single source of truth)
// ---------------------------------------------------------------------------

/// One link in a kinematic chain.
///
/// In MJCF each `<body pos="..." quat="...">` defines the rigid transform
/// from the parent joint frame to this body's origin.  If the body also
/// carries a revolute joint, `axis` is its rotation axis in the body frame
/// and `name` is the joint name used for angle lookup.
/// Fixed links (e.g. the foot-site pseudo-segment) have `axis = None`.
struct Segment {
    pos:  Vector3<f64>,
    quat: UnitQuaternion<f64>,
    axis: Option<Vector3<f64>>,
    name: &'static str,
}

/// Build a `UnitQuaternion` from MJCF [w, x, y, z] values.
#[inline]
fn q(w: f64, x: f64, y: f64, z: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(nalgebra::Quaternion::new(w, x, y, z))
}

/// Evaluate a kinematic chain given a map of joint name → angle.
/// Returns T_trunk_site: the foot-site frame expressed in the trunk frame.
fn eval_chain(chain: &[Segment], angles: &HashMap<String, f64>) -> Isometry3<f64> {
    let mut t = Isometry3::identity();
    for seg in chain {
        // Rigid body transform: translation + orientation from MJCF body
        t = t * Isometry3::from_parts(Translation3::from(seg.pos), seg.quat);
        // Revolute joint rotation (if any)
        if let Some(axis) = seg.axis {
            let angle = angles.get(seg.name).copied().unwrap_or(0.0);
            let rot = UnitQuaternion::from_scaled_axis(axis * angle);
            t = t * Isometry3::from_parts(Translation3::identity(), rot);
        }
    }
    t
}

/// Left leg: trunk_base → left_foot site.
///
/// Body chain from robot_standup.xml:
///   roll_motor_bottom (left_hip_yaw)
///   → left_roll_to_pitch (left_hip_roll)
///   → leg_plate (left_hip_pitch)
///   → leg_plate_2 (left_knee)
///   → foot (left_ankle)
///   → <site name="left_foot"> (fixed)
fn left_leg_chain() -> Vec<Segment> {
    let az = Some(Vector3::z());
    vec![
        Segment { pos: Vector3::new( 0.0125202,  0.02,    0.0208 ), quat: q( 0.0,      0.707107, -0.707107,  0.0      ), axis: az,   name: "left_hip_yaw"   },
        Segment { pos: Vector3::new( 0.0,       -0.01455, 0.0286 ), quat: q( 0.0,      0.0,      -0.707107,  0.707107 ), axis: az,   name: "left_hip_roll"  },
        Segment { pos: Vector3::new( 0.0544,     0.0,    -0.0245 ), quat: q( 0.5,      0.5,       0.5,        0.5     ), axis: az,   name: "left_hip_pitch" },
        Segment { pos: Vector3::new(-0.042,      0.0,     0.0    ), quat: q( 0.707107, 0.0,       0.0,       -0.707107), axis: az,   name: "left_knee"      },
        Segment { pos: Vector3::new( 0.0,       -0.042,   0.0    ), quat: q( 0.0,      0.0,       0.0,       -1.0     ), axis: az,   name: "left_ankle"     },
        // foot-site (fixed) — <site name="left_foot" pos="0.0048 0.021 -0.0145" quat="0.707107 0.707107 0 0"/>
        Segment { pos: Vector3::new( 0.0048,     0.021,  -0.0145 ), quat: q( 0.707107, 0.707107,  0.0,        0.0     ), axis: None, name: ""               },
    ]
}

/// Right leg: trunk_base → right_foot site.
///
/// Body chain from robot_standup.xml:
///   xl330 (right_hip_yaw)
///   → right_roll_to_pitch (right_hip_roll)
///   → xl330_2 (right_hip_pitch)
///   → xl330_3 (right_knee)
///   → foot_2 (right_ankle)
///   → <site name="right_foot"> (fixed)
fn right_leg_chain() -> Vec<Segment> {
    let az = Some(Vector3::z());
    vec![
        Segment { pos: Vector3::new( 0.0125202, -0.02,    0.0208 ), quat: q( 0.0,      -1.0,       0.0,        0.0     ), axis: az,   name: "right_hip_yaw"   },
        Segment { pos: Vector3::new( 0.01455,    0.0,     0.0286 ), quat: q( 0.5,      -0.5,       0.5,       -0.5     ), axis: az,   name: "right_hip_roll"  },
        Segment { pos: Vector3::new(-0.0544,     0.0,    -0.0245 ), quat: q( 0.5,       0.5,      -0.5,       -0.5     ), axis: az,   name: "right_hip_pitch" },
        Segment { pos: Vector3::new( 0.042,      0.0,     0.0    ), quat: q( 0.707107,  0.0,       0.0,        0.707107), axis: az,   name: "right_knee"      },
        Segment { pos: Vector3::new( 0.0,       -0.042,   0.0    ), quat: q( 1.0,       0.0,       0.0,        0.0     ), axis: az,   name: "right_ankle"     },
        // foot-site (fixed) — <site name="right_foot" pos="0.0048 -0.021 -0.0145" quat="0.707107 -0.707107 0 0"/>
        Segment { pos: Vector3::new( 0.0048,    -0.021,  -0.0145 ), quat: q( 0.707107, -0.707107,  0.0,        0.0     ), axis: None, name: ""               },
    ]
}

// ---------------------------------------------------------------------------
// Odometry
// ---------------------------------------------------------------------------

pub struct Odometry {
    /// Pre-built FK chains: [0] = left foot, [1] = right foot.
    chains: [Vec<Segment>; 2],
    /// Current joint angles by name (updated every tick).
    angles: HashMap<String, f64>,
    /// Index into `chains` for the current anchor foot (0=left, 1=right).
    anchor_chain: usize,
    /// Contact-point offset within the anchor foot-site frame.
    anchor_local: Vector3<f64>,
    /// World X,Y of the anchor contact point (Z is always 0 — flat ground).
    anchor_xy: [f64; 2],
    /// Latest trunk world position [x, y, z] in metres.
    pub position: [f64; 3],
    /// Latest trunk yaw (radians).
    pub yaw: f64,
    /// Pending contact-switch candidate (must persist for SWITCH_CONFIRM_TICKS).
    pending_switch: Option<(usize, Vector3<f64>, [f64; 2])>,
    /// How many consecutive ticks the pending candidate has been lowest.
    pending_ticks: u32,
}

impl Odometry {
    /// Build FK chains from hardcoded MJCF data.
    pub fn new() -> Self {
        let angles: HashMap<String, f64> = MOTOR_JOINTS
            .iter()
            .filter_map(|o| *o)
            .map(|n| (n.to_string(), 0.0))
            .collect();

        let left_chain  = left_leg_chain();
        let right_chain = right_leg_chain();

        // Seed z from FK at neutral pose (foot-site is at ground level)
        let t_left    = eval_chain(&left_chain, &angles);
        let initial_z = -t_left.translation.vector[2];

        Self {
            chains: [left_chain, right_chain],
            angles,
            anchor_chain: 0,
            anchor_local: Vector3::zeros(),
            anchor_xy:    [0.0, 0.0],
            position:     [0.0, 0.0, initial_z],
            yaw:          0.0,
            pending_switch: None,
            pending_ticks:  0,
        }
    }

    /// Update odometry with new sensor readings.
    ///
    /// - `joint_angles`: 15-element array in runtime motor order
    /// - `imu_quat`:     `[w, x, y, z]` trunk orientation in world frame
    pub fn update(&mut self, joint_angles: &[f64; 15], imu_quat: [f64; 4]) {
        // Refresh joint angle map
        for (idx, joint_opt) in MOTOR_JOINTS.iter().enumerate() {
            if let Some(name) = joint_opt {
                self.angles.insert(name.to_string(), joint_angles[idx]);
            }
        }

        let [qw, qx, qy, qz] = imu_quat;
        let rot = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qw, qx, qy, qz));
        let r   = rot.to_rotation_matrix();

        // Re-project trunk: current anchor sits at (anchor_xy, 0)
        self.apply_support(&r);

        // Contact switch with temporal confirmation
        match self.find_lower(&r, -SWITCH_MARGIN) {
            None => {
                self.pending_switch = None;
                self.pending_ticks  = 0;
            }
            Some((chain_idx, local, world_xy)) => {
                let same = self.pending_switch
                    .as_ref()
                    .map_or(false, |(pi, _, _)| *pi == chain_idx);
                if same {
                    self.pending_ticks += 1;
                } else {
                    self.pending_switch = Some((chain_idx, local, world_xy));
                    self.pending_ticks  = 1;
                }
                if self.pending_ticks >= SWITCH_CONFIRM_TICKS {
                    let (ci, loc, wxy) = self.pending_switch.take().unwrap();
                    self.anchor_chain  = ci;
                    self.anchor_local  = loc;
                    self.anchor_xy     = wxy;
                    self.apply_support(&r);
                    self.pending_ticks = 0;
                }
            }
        }

        let rm = r.matrix();
        self.yaw = f64::atan2(rm[(1, 0)], rm[(0, 0)]);
    }

    // ------------------------------------------------------------------

    fn apply_support(&mut self, r: &Rotation3<f64>) {
        let t       = eval_chain(&self.chains[self.anchor_chain], &self.angles);
        let p_trunk = t * Point3::from(self.anchor_local);
        let rp      = r * p_trunk.coords;
        self.position = [
            self.anchor_xy[0] - rp[0],
            self.anchor_xy[1] - rp[1],
            -rp[2],
        ];
    }

    fn find_lower(&self, r: &Rotation3<f64>, threshold: f64)
        -> Option<(usize, Vector3<f64>, [f64; 2])>
    {
        // 4 corners per foot (front/back × left/right) — used only for DETECTION.
        // When a foot wins, we anchor at its SITE CENTER (not the corner) to avoid
        // the systematic step-length bias that corner offsets introduce.
        let candidates = [
            (0usize, Vector3::new( FOOT_SOLE_HALF_LEN,  FOOT_SOLE_HALF_WIDTH, 0.0)),
            (0,      Vector3::new( FOOT_SOLE_HALF_LEN, -FOOT_SOLE_HALF_WIDTH, 0.0)),
            (0,      Vector3::new(-FOOT_SOLE_HALF_LEN,  FOOT_SOLE_HALF_WIDTH, 0.0)),
            (0,      Vector3::new(-FOOT_SOLE_HALF_LEN, -FOOT_SOLE_HALF_WIDTH, 0.0)),
            (1,      Vector3::new( FOOT_SOLE_HALF_LEN,  FOOT_SOLE_HALF_WIDTH, 0.0)),
            (1,      Vector3::new( FOOT_SOLE_HALF_LEN, -FOOT_SOLE_HALF_WIDTH, 0.0)),
            (1,      Vector3::new(-FOOT_SOLE_HALF_LEN,  FOOT_SOLE_HALF_WIDTH, 0.0)),
            (1,      Vector3::new(-FOOT_SOLE_HALF_LEN, -FOOT_SOLE_HALF_WIDTH, 0.0)),
        ];

        let trunk_pos = Vector3::from(self.position);
        let mut lower_z  = threshold;
        let mut best_chain: Option<usize> = None;

        for (chain_idx, local) in candidates {
            let t       = eval_chain(&self.chains[chain_idx], &self.angles);
            let p_trunk = t * Point3::from(local);
            let p_world = trunk_pos + r * p_trunk.coords;
            if p_world[2] < lower_z {
                lower_z    = p_world[2];
                best_chain = Some(chain_idx);
            }
        }

        // Anchor at the foot-site CENTER, not the detecting corner.
        best_chain.map(|ci| {
            let t          = eval_chain(&self.chains[ci], &self.angles);
            let site_world = trunk_pos + r * t.translation.vector;
            (ci, Vector3::zeros(), [site_world[0], site_world[1]])
        })
    }
}
