use crate::imu::ImuData;
use crate::motor::{MotorState, NUM_MOTORS, MOUTH_MOTOR_IDX, WHEEL_MOTOR_INDICES};

/// Convert a (w,x,y,z) quaternion to a 3x3 rotation matrix (row-major).
#[inline]
fn quat_to_rotmat(q: [f32; 4]) -> [[f32; 3]; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let xx = x*x; let yy = y*y; let zz = z*z;
    let xy = x*y; let xz = x*z; let yz = y*z;
    let wx = w*x; let wy = w*y; let wz = w*z;
    [
        [1.0 - 2.0*(yy+zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),     1.0 - 2.0*(xx+zz),   2.0*(yz - wx)],
        [2.0*(xz - wy),     2.0*(yz + wx),       1.0 - 2.0*(xx+yy)],
    ]
}

#[inline]
fn quat_conj(q: [f32; 4]) -> [f32; 4] { [q[0], -q[1], -q[2], -q[3]] }

#[inline]
fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ]
}

/// Compute (motion_anchor_pos_b, motion_anchor_ori_b) for the tracking observation.
///
/// Inputs are in the world frame:
///   `q_imu`               : robot trunk quaternion (w,x,y,z) from IMU
///   `ref_trunk_quat_now`  : reference trunk quaternion at current step
///   `ref_trunk_pos_now`   : reference trunk world position at current step
///   `ref_trunk_pos_t0`    : reference trunk world position at step 0 (captured on first tick)
///
/// Faking assumption: robot's world trunk position = ref_trunk_pos_t0 (i.e. robot starts where the
/// reference starts and we don't track its world translation). Orientation is real (from IMU).
/// This means motion_anchor_pos_b encodes "how far the reference has traveled from the start,
/// rotated into the robot's current body frame" — which is the forward-displacement signal the
/// policy needs. Training noise on these terms is ±0.25m / ±0.05 so the approximation is within
/// noise bounds for the 1.7s roulade duration.
pub fn compute_tracking_anchor_obs(
    q_imu: [f32; 4],
    ref_trunk_quat_now: [f32; 4],
    ref_trunk_pos_now: [f32; 3],
    ref_trunk_pos_t0: [f32; 3],
) -> ([f32; 3], [f32; 6]) {
    // World displacement of reference from start
    let dx = ref_trunk_pos_now[0] - ref_trunk_pos_t0[0];
    let dy = ref_trunk_pos_now[1] - ref_trunk_pos_t0[1];
    let dz = ref_trunk_pos_now[2] - ref_trunk_pos_t0[2];

    // Rotate into robot body frame: pos_b = R_imu^T @ [dx, dy, dz]
    let r = quat_to_rotmat(q_imu);
    let pos_b = [
        r[0][0]*dx + r[1][0]*dy + r[2][0]*dz,
        r[0][1]*dx + r[1][1]*dy + r[2][1]*dz,
        r[0][2]*dx + r[1][2]*dy + r[2][2]*dz,
    ];

    // ori_b = R_imu^T @ R_ref, keep first 2 columns, flattened row-major (matches mjlab):
    // [m00, m01, m10, m11, m20, m21]
    let q_rel = quat_mul(quat_conj(q_imu), ref_trunk_quat_now);
    let m = quat_to_rotmat(q_rel);
    let ori_b = [m[0][0], m[0][1], m[1][0], m[1][1], m[2][0], m[2][1]];

    (pos_b, ori_b)
}

/// Maximum observation size (with phase)
pub const MAX_OBSERVATION_SIZE: usize = 100;

/// Observation vector structure
///
/// Layout: [gyro(3), projected_gravity(3), joint_pos(15), joint_vel(15), last_action(15), command(3)]
/// Total: 54D  (15 joints: left leg(5) + neck/head/mouth(5) + right leg(5))
///
/// - gyro: angular velocity in body frame (rad/s)
/// - projected_gravity: normalized gravity vector in body frame (unit vector)
/// - joint_pos: relative to DEFAULT_POSITION (rad)
/// - joint_vel: joint velocities (rad/s)
/// - last_action: previous action outputs (rad offsets)
/// - command: [lin_vel_x, lin_vel_y, ang_vel_z] velocity commands
#[derive(Debug, Clone)]
pub struct Observation {
    data: [f32; MAX_OBSERVATION_SIZE],
    size: usize,
}

impl Observation {
    /// Create a new observation from sensor data.
    ///
    /// When `mouth_enabled` is true, all 15 joints are included → 54D observation.
    /// When false, the mouth joint (index MOUTH_MOTOR_IDX) is skipped → 51D observation,
    /// for backward compatibility with 14-joint policies.
    pub fn new(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
        mouth_enabled: bool,
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3)
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Projected gravity / raw accelerometer (3)
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Joint positions relative to default position
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }

        // Joint velocities
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }

        // Last action
        for i in 0..NUM_MOTORS {
            if !mouth_enabled && i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = last_action[i];
            idx += 1;
        }

        // Velocity commands (3)
        for i in 0..3 {
            data[idx] = command[i] as f32;
            idx += 1;
        }

        let size = idx;
        let expected = if mouth_enabled { 54 } else { 51 };
        assert!(size == expected, "Observation size must be {}, got {}", expected, size);

        Self { data, size }
    }

    /// Get the observation as a slice
    pub fn as_slice(&self) -> &[f32] {
        &self.data[..self.size]
    }

    /// Get the observation size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Create a 49D observation for the motorized-wheel policy.
    ///
    /// Layout: [gyro(3), accel(3), joint_pos(12), joint_vel(14), last_action(14), command(3)]
    /// - joint_pos(12): all joints except wheels (WHEEL_MOTOR_INDICES) and mouth (MOUTH_MOTOR_IDX)
    /// - joint_vel(14): all joints except mouth
    /// - last_action(14): all joints except mouth
    pub fn new_motorized_wheel(
        imu: &ImuData,
        command: &[f64; 3],
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3)
        for i in 0..3 {
            data[idx] = imu.gyro[i] as f32;
            idx += 1;
        }

        // Accelerometer / projected gravity (3)
        for i in 0..3 {
            data[idx] = imu.accel[i] as f32;
            idx += 1;
        }

        // Joint positions (12): skip wheels and mouth
        for i in 0..NUM_MOTORS {
            if WHEEL_MOTOR_INDICES.contains(&i) || i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }

        // Joint velocities (14): skip mouth only
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }

        // Last action (14): skip mouth only
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = last_action[i];
            idx += 1;
        }

        // Velocity commands (3)
        for i in 0..3 {
            data[idx] = command[i] as f32;
            idx += 1;
        }

        let size = idx;
        assert!(size == 49, "Motorized wheel observation size must be 49, got {}", size);

        Self { data, size }
    }

    /// Build a 54D observation for the kick-ball policy.
    ///
    /// Layout: [gyro(3), projected_gravity(3), joint_pos_rel(14), joint_vel(14),
    ///          last_action(14), ball_pos_body(3), kick_vel_body(3)]
    ///
    /// - `ball_pos_body`: ball position in the robot's trunk frame (m).
    ///   Pass `[0, 0, 0]` when the ball hasn't been detected yet.
    /// - `kick_vel_body`: target kick velocity in trunk frame (m/s).
    ///   For always-forward kicks use `[speed, 0, 0]`.
    pub fn new_kick_ball(
        imu: &ImuData,
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
        ball_pos_body: [f64; 3],
        kick_vel_body: [f64; 3],
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // Angular velocity (3)
        for v in &imu.gyro  { data[idx] = *v as f32; idx += 1; }
        // Projected gravity (3)
        for v in &imu.accel { data[idx] = *v as f32; idx += 1; }

        // Joint positions relative to default (14, skip mouth)
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }
        // Joint velocities (14, skip mouth)
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }
        // Last action (14, skip mouth)
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = last_action[i];
            idx += 1;
        }

        // Ball position in body frame (3)
        for v in &ball_pos_body  { data[idx] = *v as f32; idx += 1; }
        // Target kick velocity in body frame (3)
        for v in &kick_vel_body  { data[idx] = *v as f32; idx += 1; }

        let size = idx;
        assert!(size == 54, "Kick-ball observation size must be 54, got {}", size);
        Self { data, size }
    }

    /// Get mutable access to the observation data
    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.data[..self.size]
    }

    /// Build the 85D tracking observation matching mjlab.tasks.tracking policy inputs.
    ///
    /// Layout (order must match mjlab/tasks/tracking/tracking_env_cfg.py policy_terms):
    ///   command (28)              : ref_joint_pos (14) || ref_joint_vel (14)
    ///   motion_anchor_pos_b (3)   : reference trunk pos in robot's trunk frame
    ///   motion_anchor_ori_b (6)   : first 2 cols of (R_imu^T · R_ref)
    ///   base_lin_vel (3)          : IMU lin vel (body frame) — we pass [0,0,0] since we lack it
    ///   base_ang_vel (3)          : gyro
    ///   joint_pos (14)            : motor_state.positions - default (mouth excluded)
    ///   joint_vel (14)            : motor_state.velocities (mouth excluded)
    ///   actions (14)              : last_action (mouth excluded)
    ///
    /// `ref_joint_pos` / `ref_joint_vel` must be in mjlab hinge-joint order (14 slots).
    /// The caller is responsible for remapping to/from runtime motor order.
    pub fn new_tracking(
        imu: &ImuData,
        motor_state: &MotorState,
        last_action: &[f32; NUM_MOTORS],
        default_positions: &[f64; NUM_MOTORS],
        ref_joint_pos: &[f32; 14],
        ref_joint_vel: &[f32; 14],
        motion_anchor_pos_b: &[f32; 3],
        motion_anchor_ori_b: &[f32; 6],
    ) -> Self {
        let mut data = [0.0f32; MAX_OBSERVATION_SIZE];
        let mut idx = 0;

        // command: ref_joint_pos (14) then ref_joint_vel (14)
        for v in ref_joint_pos { data[idx] = *v; idx += 1; }
        for v in ref_joint_vel { data[idx] = *v; idx += 1; }

        // motion_anchor_pos_b (3)
        for v in motion_anchor_pos_b { data[idx] = *v; idx += 1; }
        // motion_anchor_ori_b (6)
        for v in motion_anchor_ori_b { data[idx] = *v; idx += 1; }

        // base_lin_vel (3) — not available on hardware; we pass zeros.
        // Training adds Unoise(-0.5, 0.5) so the policy should tolerate this.
        for _ in 0..3 { data[idx] = 0.0; idx += 1; }

        // base_ang_vel (3): gyro
        for i in 0..3 { data[idx] = imu.gyro[i] as f32; idx += 1; }

        // joint_pos (14): relative to default, mouth excluded
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = (motor_state.positions[i] - default_positions[i]) as f32;
            idx += 1;
        }
        // joint_vel (14): mouth excluded
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = motor_state.velocities[i] as f32;
            idx += 1;
        }
        // actions (14): last action, mouth excluded
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { continue; }
            data[idx] = last_action[i];
            idx += 1;
        }

        let size = idx;
        assert!(size == 85, "Tracking observation size must be 85, got {}", size);
        Self { data, size }
    }
}

impl Default for Observation {
    fn default() -> Self {
        Self {
            data: [0.0; MAX_OBSERVATION_SIZE],
            size: 51,  // Default: legacy 14-joint mode
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::{MotorState, DEFAULT_POSITION};

    #[test]
    fn test_observation_size_legacy() {
        let obs = Observation::default();
        assert_eq!(obs.as_slice().len(), 51);
        assert_eq!(obs.size(), 51);
    }

    #[test]
    fn test_observation_creation_legacy() {
        let imu = ImuData { gyro: [1.0, 2.0, 3.0], accel: [4.0, 5.0, 6.0], ..ImuData::default() };
        let command = [0.1, 0.2, 0.3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, &DEFAULT_POSITION, false);
        let slice = obs.as_slice();

        assert_eq!(obs.size(), 51);
        assert_eq!(slice[0], 1.0); // gyro x
        assert_eq!(slice[3], 4.0); // accel x
        // command at end: 3 + 3 + 14 + 14 + 14 = 48
        assert_eq!(slice[48], 0.1);
        assert_eq!(slice[49], 0.2);
        assert_eq!(slice[50], 0.3);
    }

    #[test]
    fn test_observation_creation_mouth() {
        let imu = ImuData { gyro: [1.0, 2.0, 3.0], accel: [4.0, 5.0, 6.0], ..ImuData::default() };
        let command = [0.1, 0.2, 0.3];
        let motor_state = MotorState::default();
        let last_action = [0.0f32; NUM_MOTORS];

        let obs = Observation::new(&imu, &command, &motor_state, &last_action, &DEFAULT_POSITION, true);
        let slice = obs.as_slice();

        assert_eq!(obs.size(), 54);
        // command at end: 3 + 3 + 15 + 15 + 15 = 51
        assert_eq!(slice[51], 0.1);
        assert_eq!(slice[52], 0.2);
        assert_eq!(slice[53], 0.3);
    }

}
