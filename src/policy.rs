use anyhow::Result;
use crate::motor::{NUM_MOTORS, MOUTH_MOTOR_IDX};
use crate::observation::Observation;
use ort::session::{Session, builder::GraphOptimizationLevel};
use ort::value::Value;
use std::time::Instant;

/// Policy mode
pub enum PolicyMode {
    /// Dummy policy that generates squatting motion
    Dummy,
    /// ONNX Runtime policy
    Onnx(Session),
}

/// Cached reference tensors returned by the tracking ONNX for the current time_step.
/// The policy runs the ONNX once, caches the refs, then the next observation is built
/// from them. Field sizes match the motion baked into the model:
/// - joint_pos / joint_vel : 14 (hinge joints)
/// - body_pos_w / body_quat_w : [NB, 3] / [NB, 4] (world-frame body pose, NB = len(body_names))
/// - body_lin_vel_w / body_ang_vel_w : [NB, 3]
#[derive(Default, Clone)]
pub struct RoulaideRefs {
    pub joint_pos: [f32; 14],
    pub joint_vel: [f32; 14],
    /// [nbody, 3] flattened. Size set from first ONNX call.
    pub body_pos_w: Vec<f32>,
    pub body_quat_w: Vec<f32>,
    pub body_lin_vel_w: Vec<f32>,
    pub body_ang_vel_w: Vec<f32>,
    pub num_bodies: usize,
    /// Whether the refs have been populated at least once (by running ONNX at t=0).
    pub initialized: bool,
}

/// Policy inference engine using ONNX Runtime
pub struct Policy {
    mode: PolicyMode,
    standing_mode: Option<PolicyMode>,
    ground_pick_mode: Option<PolicyMode>,
    ground_pick_active: bool,
    jump_mode: Option<PolicyMode>,
    jump_active: bool,
    roulade_session: Option<Session>,
    roulade_active: bool,
    roulade_refs: RoulaideRefs,
    /// Precomputed refs for every timestep of the motion. Populated once on
    /// activation so the hot path only needs one ONNX call per tick (just for
    /// the action) instead of two.
    roulade_ref_table: Vec<RoulaideRefs>,
    fold_mode: Option<PolicyMode>,
    start_time: Instant,
    command_threshold: f64,
    standing_disabled: bool,
}

impl Policy {
    /// Create a new dummy policy that generates squatting motion
    pub fn new_dummy() -> Result<Self> {
        Ok(Self {
            mode: PolicyMode::Dummy,
            standing_mode: None,
            ground_pick_mode: None,
            ground_pick_active: false,
            jump_mode: None,
            jump_active: false,
            roulade_session: None,
            roulade_active: false,
            roulade_refs: RoulaideRefs::default(),
            roulade_ref_table: Vec::new(),
            fold_mode: None,
            start_time: Instant::now(),
            command_threshold: 0.05,
            standing_disabled: false,
        })
    }

    /// Create a new policy from an ONNX model file
    ///
    /// # Arguments
    /// * `model_path` - Path to the ONNX model file
    pub fn new_onnx(model_path: &str) -> Result<Self> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)? // Limit threads for Raspberry Pi Zero 2W
            .commit_from_file(model_path)?;

        Ok(Self {
            mode: PolicyMode::Onnx(session),
            standing_mode: None,
            ground_pick_mode: None,
            ground_pick_active: false,
            jump_mode: None,
            jump_active: false,
            roulade_session: None,
            roulade_active: false,
            roulade_refs: RoulaideRefs::default(),
            roulade_ref_table: Vec::new(),
            fold_mode: None,
            start_time: Instant::now(),
            command_threshold: 0.05,
            standing_disabled: false,
        })
    }

    /// Load a standing policy from an ONNX model file
    pub fn add_standing(&mut self, path: &str) -> Result<()> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        self.standing_mode = Some(PolicyMode::Onnx(session));
        Ok(())
    }

    /// Load a ground pick policy from an ONNX model file
    pub fn add_ground_pick(&mut self, path: &str) -> Result<()> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        self.ground_pick_mode = Some(PolicyMode::Onnx(session));
        Ok(())
    }

    /// Activate or deactivate ground pick mode
    pub fn set_ground_pick_active(&mut self, active: bool) {
        self.ground_pick_active = active;
    }

    /// Whether a ground pick policy is loaded
    pub fn has_ground_pick(&self) -> bool {
        self.ground_pick_mode.is_some()
    }

    /// Load a jump policy from an ONNX model file
    pub fn add_jump(&mut self, path: &str) -> Result<()> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        self.jump_mode = Some(PolicyMode::Onnx(session));
        Ok(())
    }

    /// Activate or deactivate jump mode
    pub fn set_jump_active(&mut self, active: bool) {
        self.jump_active = active;
    }

    /// Whether a jump policy is loaded
    pub fn has_jump(&self) -> bool {
        self.jump_mode.is_some()
    }

    /// Load a roulade (motion-tracking) policy from an ONNX model file.
    ///
    /// The model is expected to have two inputs `obs` [1,85] (f32) and `time_step` [1,1] (f32),
    /// and seven outputs: actions [1,14], joint_pos [1,14], joint_vel [1,14],
    /// body_pos_w [1,NB,3], body_quat_w [1,NB,4], body_lin_vel_w [1,NB,3], body_ang_vel_w [1,NB,3].
    /// The motion data is baked inside the ONNX (see mjlab tracking exporter).
    pub fn add_roulade(&mut self, path: &str) -> Result<()> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        self.roulade_session = Some(session);
        self.roulade_refs = RoulaideRefs::default();
        Ok(())
    }

    pub fn has_roulade(&self) -> bool { self.roulade_session.is_some() }

    pub fn set_roulade_active(&mut self, active: bool) {
        self.roulade_active = active;
        if active {
            // Reset cached refs so the first step uses zeros until we've run ONNX once.
            self.roulade_refs.initialized = false;
        }
    }

    pub fn roulade_refs(&self) -> &RoulaideRefs { &self.roulade_refs }

    /// Precomputed refs for the given time_step (clamped into range). Empty
    /// default returned if the table hasn't been populated.
    pub fn roulade_refs_at(&self, time_step: i64) -> Option<&RoulaideRefs> {
        if self.roulade_ref_table.is_empty() {
            return None;
        }
        let last = self.roulade_ref_table.len() as i64 - 1;
        let idx = time_step.clamp(0, last) as usize;
        Some(&self.roulade_ref_table[idx])
    }

    /// Precompute refs for all timesteps 0..num_steps by running the baked
    /// motion ONNX with a dummy observation — the refs don't depend on obs.
    /// One-time cost at roulade activation (~num_steps inferences), but then
    /// the hot path avoids the "bootstrap refs" inference each tick.
    pub fn precompute_roulade_refs(&mut self, num_steps: i64) -> Result<()> {
        if num_steps <= 0 { return Ok(()); }
        let session = self.roulade_session.as_mut()
            .ok_or_else(|| anyhow::anyhow!("roulade policy not loaded"))?;

        // Build a zero-obs once (observations ignored by the ref outputs).
        let obs_len = 85usize;
        let obs_zero = vec![0.0f32; obs_len];

        let mut table: Vec<RoulaideRefs> = Vec::with_capacity(num_steps as usize);
        for t in 0..num_steps {
            let obs_value = Value::from_array(([1usize, obs_len], obs_zero.clone()))
                .map_err(|e| anyhow::anyhow!("obs value: {}", e))?;
            let ts_value = Value::from_array(([1usize, 1usize], vec![t as f32]))
                .map_err(|e| anyhow::anyhow!("ts value: {}", e))?;
            let outputs = session
                .run(ort::inputs!["obs" => &obs_value, "time_step" => &ts_value])
                .map_err(|e| anyhow::anyhow!("precompute inference failed: {}", e))?;

            let get = |name: &str| -> Result<(Vec<i64>, Vec<f32>)> {
                let v = outputs.get(name)
                    .ok_or_else(|| anyhow::anyhow!("missing output '{}'", name))?;
                let (shape, data) = v.try_extract_tensor::<f32>()
                    .map_err(|e| anyhow::anyhow!("extract '{}': {}", name, e))?;
                Ok((shape.as_ref().to_vec(), data.to_vec()))
            };
            let (_, jp) = get("joint_pos")?;
            let (_, jv) = get("joint_vel")?;
            let (bp_shape, bp) = get("body_pos_w")?;
            let (_, bq) = get("body_quat_w")?;
            let (_, blv) = get("body_lin_vel_w")?;
            let (_, bav) = get("body_ang_vel_w")?;
            let num_bodies = if bp_shape.len() == 3 { bp_shape[1] as usize } else { bp.len() / 3 };

            let mut refs = RoulaideRefs::default();
            for i in 0..14 {
                refs.joint_pos[i] = jp[i];
                refs.joint_vel[i] = jv[i];
            }
            refs.body_pos_w = bp;
            refs.body_quat_w = bq;
            refs.body_lin_vel_w = blv;
            refs.body_ang_vel_w = bav;
            refs.num_bodies = num_bodies;
            refs.initialized = true;
            table.push(refs);
        }
        self.roulade_ref_table = table;
        Ok(())
    }

    /// Infer action only — assumes the caller has already built `observation`
    /// from the precomputed ref table. Single ONNX call per tick.
    pub fn infer_roulade_action_only(
        &mut self,
        observation: &Observation,
        time_step: i64,
    ) -> Result<[f32; NUM_MOTORS]> {
        let session = self.roulade_session.as_mut()
            .ok_or_else(|| anyhow::anyhow!("roulade policy not loaded"))?;
        let obs_vec = observation.as_slice().to_vec();
        let obs_size = obs_vec.len();
        let obs_value = Value::from_array(([1usize, obs_size], obs_vec))
            .map_err(|e| anyhow::anyhow!("obs value: {}", e))?;
        let ts_value = Value::from_array(([1usize, 1usize], vec![time_step as f32]))
            .map_err(|e| anyhow::anyhow!("ts value: {}", e))?;
        let outputs = session
            .run(ort::inputs!["obs" => &obs_value, "time_step" => &ts_value])
            .map_err(|e| anyhow::anyhow!("Roulade ONNX inference failed: {}", e))?;
        let v = outputs.get("actions")
            .ok_or_else(|| anyhow::anyhow!("roulade ONNX missing 'actions'"))?;
        let (shape, data) = v.try_extract_tensor::<f32>()
            .map_err(|e| anyhow::anyhow!("extract actions: {}", e))?;
        let shape = shape.as_ref();
        if shape.len() != 2 || shape[1] != 14 {
            return Err(anyhow::anyhow!("Roulade actions shape {:?} != [1,14]", shape));
        }
        let mut actions = [0.0f32; NUM_MOTORS];
        let mut out_idx = 0;
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { actions[i] = 0.0; }
            else { actions[i] = data[out_idx]; out_idx += 1; }
        }
        Ok(actions)
    }

    /// Run the tracking ONNX for one step.
    ///
    /// `observation` must be the 85D tracking obs (see `Observation::new_tracking`).
    /// `time_step` is the integer frame index into the motion (clamped by the ONNX at max-1).
    /// Returns the 14 action offsets and updates `self.roulade_refs` so the caller can
    /// build the next observation.
    pub fn infer_roulade(
        &mut self,
        observation: &Observation,
        time_step: i64,
    ) -> Result<[f32; NUM_MOTORS]> {
        let session = self.roulade_session.as_mut()
            .ok_or_else(|| anyhow::anyhow!("roulade policy not loaded"))?;

        let obs_vec = observation.as_slice().to_vec();
        let obs_size = obs_vec.len();
        let obs_value = Value::from_array(([1usize, obs_size], obs_vec))
            .map_err(|e| anyhow::anyhow!("Failed to create obs value: {}", e))?;
        let ts_value = Value::from_array(([1usize, 1usize], vec![time_step as f32]))
            .map_err(|e| anyhow::anyhow!("Failed to create time_step value: {}", e))?;

        let outputs = session
            .run(ort::inputs!["obs" => &obs_value, "time_step" => &ts_value])
            .map_err(|e| anyhow::anyhow!("Roulade ONNX inference failed: {}", e))?;

        // Outputs are keyed by name (see exporter.py output_names). Extract by name.
        let get = |name: &str| -> Result<(Vec<i64>, Vec<f32>)> {
            let v = outputs.get(name)
                .ok_or_else(|| anyhow::anyhow!("roulade ONNX missing output '{}'", name))?;
            let (shape, data) = v.try_extract_tensor::<f32>()
                .map_err(|e| anyhow::anyhow!("Failed to extract '{}': {}", name, e))?;
            Ok((shape.as_ref().to_vec(), data.to_vec()))
        };

        let (actions_shape, actions_data) = get("actions")?;
        if actions_shape.len() != 2 || actions_shape[1] != 14 {
            return Err(anyhow::anyhow!(
                "Roulade actions shape {:?} != [1,14]", actions_shape));
        }

        let (_, jp) = get("joint_pos")?;
        let (_, jv) = get("joint_vel")?;
        let (bp_shape, bp) = get("body_pos_w")?;
        let (_, bq) = get("body_quat_w")?;
        let (_, blv) = get("body_lin_vel_w")?;
        let (_, bav) = get("body_ang_vel_w")?;

        // Determine number of bodies from body_pos_w shape [1, NB, 3]
        let num_bodies = if bp_shape.len() == 3 { bp_shape[1] as usize } else { bp.len() / 3 };

        // Cache refs. joint_pos/vel are [1,14] → copy to fixed arrays.
        for i in 0..14 {
            self.roulade_refs.joint_pos[i] = jp[i];
            self.roulade_refs.joint_vel[i] = jv[i];
        }
        self.roulade_refs.body_pos_w = bp;
        self.roulade_refs.body_quat_w = bq;
        self.roulade_refs.body_lin_vel_w = blv;
        self.roulade_refs.body_ang_vel_w = bav;
        self.roulade_refs.num_bodies = num_bodies;
        self.roulade_refs.initialized = true;

        // Map 14 outputs → 15-slot action array (mouth stays at 0 — roulade is legacy 14-DOF).
        let mut actions = [0.0f32; NUM_MOTORS];
        let mut out_idx = 0;
        for i in 0..NUM_MOTORS {
            if i == MOUTH_MOTOR_IDX { actions[i] = 0.0; }
            else { actions[i] = actions_data[out_idx]; out_idx += 1; }
        }
        Ok(actions)
    }

    /// Load a fold policy from an ONNX model file
    pub fn add_fold(&mut self, path: &str) -> Result<()> {
        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_intra_threads(2)?
            .commit_from_file(path)?;
        self.fold_mode = Some(PolicyMode::Onnx(session));
        Ok(())
    }

    /// Whether a fold policy is loaded
    pub fn has_fold(&self) -> bool {
        self.fold_mode.is_some()
    }

    /// Calculate command magnitude
    fn command_magnitude(command: &[f64; 3]) -> f64 {
        (command[0].powi(2) + command[1].powi(2) + command[2].powi(2)).sqrt()
    }

    /// Disable standing policy switching (e.g. roller mode)
    pub fn set_standing_disabled(&mut self, disabled: bool) {
        self.standing_disabled = disabled;
    }

    /// Returns true if the standing policy would be selected for the given command
    pub fn will_use_standing(&self, command: &[f64; 3]) -> bool {
        !self.standing_disabled && self.standing_mode.is_some() && Self::command_magnitude(command) <= self.command_threshold
    }

    /// Run inference on an observation to get actions
    ///
    /// # Arguments
    /// * `observation` - The observation vector (51 dimensions)
    /// * `command` - The velocity command [vel_x, vel_y, vel_z]
    ///
    /// # Returns
    /// Action vector (14 dimensions) for motor position offsets
    pub fn infer(&mut self, observation: &Observation, command: &[f64; 3], mouth_enabled: bool) -> Result<[f32; NUM_MOTORS]> {
        // Fold takes highest priority when loaded — always active (phase cmd controls stand vs fold)
        if let Some(ref mut fold_mode) = self.fold_mode {
            return Self::run_mode(fold_mode, observation, self.start_time, false);
        }
        // Ground pick takes priority when active
        if self.ground_pick_active {
            if let Some(ref mut gp_mode) = self.ground_pick_mode {
                return Self::run_mode(gp_mode, observation, self.start_time, false); // ground pick always legacy 14-DOF
            }
        }
        // Jump takes priority when active
        if self.jump_active {
            if let Some(ref mut jump_mode) = self.jump_mode {
                return Self::run_mode(jump_mode, observation, self.start_time, false);
            }
        }

        // Determine which policy to use based on command magnitude
        let use_standing = if self.standing_mode.is_some() && !self.standing_disabled {
            Self::command_magnitude(command) <= self.command_threshold
        } else {
            false
        };

        let policy_mode = if use_standing {
            self.standing_mode.as_mut().unwrap()
        } else {
            &mut self.mode
        };

        Self::run_mode(policy_mode, observation, self.start_time, mouth_enabled)
    }

    fn run_mode(policy_mode: &mut PolicyMode, observation: &Observation, start_time: Instant, mouth_enabled: bool) -> Result<[f32; NUM_MOTORS]> {
        match policy_mode {
            PolicyMode::Dummy => {
                // Dummy policy: generate squatting motion
                // Frequency: 0.5 Hz, Amplitude: 0.2 rad (hip_pitch, ankle), 0.4 rad (knee)
                let elapsed = start_time.elapsed().as_secs_f32();
                let freq = 0.5; // Hz
                let amplitude = 0.2; // radians
                let knee_amplitude = 0.4; // 2x amplitude for knees

                // sin(2*pi*f*t)
                let phase = (2.0 * std::f32::consts::PI * freq * elapsed).sin();

                let mut actions = [0.0f32; NUM_MOTORS];

                // Left leg
                actions[2] = amplitude * phase;         // left_hip_pitch (increases when squatting)
                actions[3] = -knee_amplitude * phase;   // left_knee (decreases when squatting)
                actions[4] = amplitude * phase;         // left_ankle (increases when squatting)

                // Right leg (mirrors left leg due to motor orientations)
                actions[11] = -amplitude * phase;       // right_hip_pitch (decreases when squatting)
                actions[12] = knee_amplitude * phase;   // right_knee (increases when squatting)
                actions[13] = -amplitude * phase;       // right_ankle (decreases when squatting)

                Ok(actions)
            }
            PolicyMode::Onnx(session) => {
                // Create input tensor from observation (shape: [1, obs_size])
                let obs_vec = observation.as_slice().to_vec();
                let obs_size = obs_vec.len();

                // Create Value from tuple (shape, data)
                // Shape is [1, obs_size] - batch size 1, observation dimension (51 or 53)
                let input_value = Value::from_array(([1, obs_size], obs_vec))
                    .map_err(|e| anyhow::anyhow!("Failed to create input value: {}", e))?;

                // Run inference
                // Note: Input name might need to be adjusted based on the ONNX model
                // Common names: "obs", "input", "observation"
                let outputs = session
                    .run(ort::inputs!["obs" => &input_value])
                    .map_err(|e| anyhow::anyhow!("ONNX inference failed (try checking input name): {}", e))?;

                // Extract output tensor (expecting shape [1, 14])
                // Try to get the first output (or by name if we knew it)
                let output = outputs.values().next()
                    .ok_or_else(|| anyhow::anyhow!("No output from ONNX model"))?;

                let (output_shape, output_data) = output
                    .try_extract_tensor::<f32>()
                    .map_err(|e| anyhow::anyhow!("Failed to extract output tensor: {}", e))?;

                // Verify output shape — accept 14 (legacy, no mouth) or 15 (mouth mode)
                let shape_dims = output_shape.as_ref();
                let expected_actions: usize = if mouth_enabled { 15 } else { 14 };
                if shape_dims.len() != 2 || shape_dims[1] != expected_actions as i64 {
                    let actual = if shape_dims.len() >= 2 { shape_dims[1] } else { -1 };
                    if !mouth_enabled && actual == 15 {
                        return Err(anyhow::anyhow!(
                            "Policy outputs 15 actions (mouth DOF included) but --mouth was not set. \
                             Add --mouth to use this policy, or load a 14-DOF policy for legacy mode."
                        ));
                    } else if mouth_enabled && actual == 14 {
                        return Err(anyhow::anyhow!(
                            "Policy outputs 14 actions (no mouth DOF) but --mouth was set. \
                             Remove --mouth to run this policy in legacy mode."
                        ));
                    }
                    return Err(anyhow::anyhow!(
                        "Unexpected policy output shape: {:?}, expected [1, {}]",
                        shape_dims, expected_actions
                    ));
                }

                // Copy output to actions array.
                // In legacy mode (14 outputs): insert 0 at MOUTH_MOTOR_IDX so the
                // rest of the pipeline always works with a 15-element array.
                let mut actions = [0.0f32; NUM_MOTORS];
                if mouth_enabled {
                    for i in 0..NUM_MOTORS {
                        actions[i] = output_data[i];
                    }
                } else {
                    // Map 14 outputs → 15 slots, skipping MOUTH_MOTOR_IDX (mouth stays at 0 = default)
                    let mut out_idx = 0;
                    for i in 0..NUM_MOTORS {
                        if i == MOUTH_MOTOR_IDX {
                            actions[i] = 0.0; // overridden by trigger below
                        } else {
                            actions[i] = output_data[out_idx];
                            out_idx += 1;
                        }
                    }
                }

                Ok(actions)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dummy_policy() {
        let mut policy = Policy::new_dummy().unwrap();
        let obs = Observation::default();
        let command = [0.0, 0.0, 0.0];
        let action = policy.infer(&obs, &command).unwrap();

        assert_eq!(action.len(), NUM_MOTORS);

        // Dummy policy generates squatting motion, so not all zeros
        // Just check that leg joints have non-zero values at some point
        // (at t=0, sin(0)=0, so we might get zeros initially)
    }

    #[test]
    fn test_dummy_policy_squat_motion() {
        use std::thread;
        use std::time::Duration;

        let mut policy = Policy::new_dummy().unwrap();
        let obs = Observation::default();
        let command = [0.0, 0.0, 0.0];

        // Wait a bit so we're not at t=0
        thread::sleep(Duration::from_millis(100));

        let action = policy.infer(&obs, &command).unwrap();

        // Check that leg joints have some motion (not all zeros)
        let has_motion = action[2] != 0.0 || action[3] != 0.0 || action[4] != 0.0 ||
                        action[11] != 0.0 || action[12] != 0.0 || action[13] != 0.0;
        assert!(has_motion, "Dummy policy should generate squat motion");
    }
}
