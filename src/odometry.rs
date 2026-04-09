//! Contact-based odometry for microduck, computed inline in Rust.
//!
//! Kinematic chains are built at startup by parsing the URDF — no hardcoded
//! transforms. If the URDF changes, the code adapts automatically.
//!
//! Algorithm (ported from Rhoban humanoid model_service.cpp):
//!   - 4 candidate contact points: front+back of each foot sole
//!   - Each step: find the point with the lowest world Z
//!   - If lower than the current anchor (which sits at z≈0), switch to it
//!   - Re-project trunk: anchor at flat ground (z=0), oriented by IMU quat

use anyhow::{Context, Result};
use nalgebra::{Isometry3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use std::collections::HashMap;
use urdf_rs::{JointType, Robot};

// ---------------------------------------------------------------------------
// Motor → joint mapping (runtime motor vector order)
// ---------------------------------------------------------------------------

/// Motor index → URDF joint name. Index 9 (mouth) has no kinematic joint.
const MOTOR_JOINTS: [Option<&str>; 15] = [
    Some("left_hip_yaw"),   // 0
    Some("left_hip_roll"),  // 1
    Some("left_hip_pitch"), // 2
    Some("left_knee"),      // 3
    Some("left_ankle"),     // 4
    Some("neck_pitch"),     // 5
    Some("head_pitch"),     // 6
    Some("head_yaw"),       // 7
    Some("head_roll"),      // 8
    None,                   // 9 — mouth, no kinematic effect
    Some("right_hip_yaw"),  // 10
    Some("right_hip_roll"), // 11
    Some("right_hip_pitch"),// 12
    Some("right_knee"),     // 13
    Some("right_ankle"),    // 14
];

// ---------------------------------------------------------------------------
// Contact geometry
// ---------------------------------------------------------------------------

/// Half-length of the duck foot sole along foot-frame X (front/back of sole).
/// Derived from foot_tpu_bottom.stl: sole is 51mm long → half = 25.5mm.
const FOOT_SOLE_HALF_LEN: f64 = 0.0255;

/// A new contact point must be this many metres BELOW the anchor to take over.
/// Prevents micro-switching on flat ground.
const SWITCH_MARGIN: f64 = 0.003;

/// URDF joint names for the left / right foot frames (fixed joints).
const LEFT_FOOT_JOINT: &str = "left_foot_frame";
const RIGHT_FOOT_JOINT: &str = "right_foot_frame";

// ---------------------------------------------------------------------------
// Kinematic chain — built from URDF at startup
// ---------------------------------------------------------------------------

struct Segment {
    name: String,
    /// Fixed transform from parent joint frame to this joint's origin.
    origin: Isometry3<f64>,
    /// Rotation axis in joint frame (zero for fixed joints).
    axis: Vector3<f64>,
    is_revolute: bool,
}

/// Build the kinematic chain from `root_link` to the child frame of
/// `target_joint`, by walking backwards through parent links.
fn build_chain(joints: &[urdf_rs::Joint], root_link: &str, target_joint: &str) -> Result<Vec<Segment>> {
    let tgt = joints
        .iter()
        .find(|j| j.name == target_joint)
        .with_context(|| format!("Joint '{}' not found in URDF", target_joint))?;

    let mut chain = vec![make_segment(tgt)];
    let mut parent = tgt.parent.link.clone();

    loop {
        if parent == root_link {
            break;
        }
        let joint = joints
            .iter()
            .find(|j| j.child.link == parent)
            .with_context(|| format!("No joint found with child link '{}'", parent))?;
        parent = joint.parent.link.clone();
        chain.push(make_segment(joint));
    }

    chain.reverse(); // root → foot order
    Ok(chain)
}

fn make_segment(joint: &urdf_rs::Joint) -> Segment {
    let o = &joint.origin;
    let origin = Isometry3::from_parts(
        Translation3::new(o.xyz[0], o.xyz[1], o.xyz[2]),
        UnitQuaternion::from_euler_angles(o.rpy[0], o.rpy[1], o.rpy[2]),
    );
    let is_revolute = matches!(
        joint.joint_type,
        JointType::Revolute | JointType::Continuous
    );
    let axis = if is_revolute {
        Vector3::new(joint.axis.xyz[0], joint.axis.xyz[1], joint.axis.xyz[2]).normalize()
    } else {
        Vector3::zeros()
    };
    Segment { name: joint.name.clone(), origin, axis, is_revolute }
}

/// Evaluate a chain given a map of joint name → angle.
/// Returns T_root_foot (transform of the foot frame in the root/trunk frame).
fn eval_chain(chain: &[Segment], angles: &HashMap<String, f64>) -> Isometry3<f64> {
    let mut t = Isometry3::identity();
    for seg in chain {
        t = t * seg.origin;
        if seg.is_revolute {
            let angle = angles.get(&seg.name).copied().unwrap_or(0.0);
            let rot = UnitQuaternion::from_scaled_axis(seg.axis * angle);
            t = t * Isometry3::from_parts(Translation3::identity(), rot);
        }
    }
    t
}

// ---------------------------------------------------------------------------
// Odometry
// ---------------------------------------------------------------------------

pub struct Odometry {
    /// Pre-built FK chains: [0] = left_foot, [1] = right_foot
    chains: [Vec<Segment>; 2],
    /// Current joint angles by name (updated every step).
    angles: HashMap<String, f64>,
    /// Index into `chains` for the current anchor foot (0=left, 1=right).
    anchor_chain: usize,
    /// Contact point offset within the anchor foot frame.
    anchor_local: Vector3<f64>,
    /// World X,Y of the anchor contact point (Z is always 0 — flat ground).
    anchor_xy: [f64; 2],
    /// Latest trunk world position [x, y, z] in metres.
    pub position: [f64; 3],
    /// Latest trunk yaw (radians).
    pub yaw: f64,
}

impl Odometry {
    /// Load the URDF and build FK chains. Fails if the file is not found or
    /// the expected joints are missing.
    pub fn new(urdf_path: &str) -> Result<Self> {
        let robot: Robot = urdf_rs::read_file(urdf_path)
            .with_context(|| format!("Failed to read URDF '{}'", urdf_path))?;

        let left_chain = build_chain(&robot.joints, "trunk_base", LEFT_FOOT_JOINT)?;
        let right_chain = build_chain(&robot.joints, "trunk_base", RIGHT_FOOT_JOINT)?;

        let angles: HashMap<String, f64> = MOTOR_JOINTS
            .iter()
            .filter_map(|o| *o)
            .map(|n| (n.to_string(), 0.0))
            .collect();

        // Seed z from FK at neutral pose (feet at z=0, flat ground)
        let t_left = eval_chain(&left_chain, &angles);
        let initial_z = -t_left.translation.vector[2];

        Ok(Self {
            chains: [left_chain, right_chain],
            angles,
            anchor_chain: 0,
            anchor_local: Vector3::zeros(),
            anchor_xy: [0.0, 0.0],
            position: [0.0, 0.0, initial_z],
            yaw: 0.0,
        })
    }

    /// Update odometry with new sensor readings.
    ///
    /// - `joint_angles`: 15-element array in runtime motor order
    /// - `imu_quat`: `[w, x, y, z]` trunk orientation in world frame
    pub fn update(&mut self, joint_angles: &[f64; 15], imu_quat: [f64; 4]) {
        // Update joint angle map
        for (idx, joint_opt) in MOTOR_JOINTS.iter().enumerate() {
            if let Some(name) = joint_opt {
                self.angles.insert(name.to_string(), joint_angles[idx]);
            }
        }

        let [qw, qx, qy, qz] = imu_quat;
        let rot = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qw, qx, qy, qz));
        let r = rot.to_rotation_matrix();

        // Re-project trunk: current anchor sits at (anchor_xy, 0)
        self.apply_support(&r);

        // Find if any contact point went below the anchor (which is at z≈0)
        if let Some((chain_idx, local, world_xy)) = self.find_lower(&r, -SWITCH_MARGIN) {
            self.anchor_chain = chain_idx;
            self.anchor_local = local;
            self.anchor_xy = world_xy;
            self.apply_support(&r);
        }

        let rm = r.matrix();
        self.yaw = f64::atan2(rm[(1, 0)], rm[(0, 0)]);
    }

    // ------------------------------------------------------------------

    fn apply_support(&mut self, r: &Rotation3<f64>) {
        let t = eval_chain(&self.chains[self.anchor_chain], &self.angles);
        let p_trunk = t * Point3::from(self.anchor_local);
        let rp = r * p_trunk.coords;
        self.position = [
            self.anchor_xy[0] - rp[0],
            self.anchor_xy[1] - rp[1],
            -rp[2],
        ];
    }

    fn find_lower(&self, r: &Rotation3<f64>, threshold: f64)
        -> Option<(usize, Vector3<f64>, [f64; 2])>
    {
        let candidates = [
            (0usize, Vector3::new( FOOT_SOLE_HALF_LEN, 0.0, 0.0)),
            (0,      Vector3::new(-FOOT_SOLE_HALF_LEN, 0.0, 0.0)),
            (1,      Vector3::new( FOOT_SOLE_HALF_LEN, 0.0, 0.0)),
            (1,      Vector3::new(-FOOT_SOLE_HALF_LEN, 0.0, 0.0)),
        ];

        let trunk_pos = Vector3::from(self.position);
        let mut lower_z = threshold;
        let mut best: Option<(usize, Vector3<f64>, [f64; 2])> = None;

        for (chain_idx, local) in candidates {
            let t = eval_chain(&self.chains[chain_idx], &self.angles);
            let p_trunk = t * Point3::from(local);
            let p_world = trunk_pos + r * p_trunk.coords;
            if p_world[2] < lower_z {
                lower_z = p_world[2];
                best = Some((chain_idx, local, [p_world[0], p_world[1]]));
            }
        }
        best
    }
}
