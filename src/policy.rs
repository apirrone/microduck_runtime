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

/// Policy inference engine using ONNX Runtime
pub struct Policy {
    mode: PolicyMode,
    standing_mode: Option<PolicyMode>,
    ground_pick_mode: Option<PolicyMode>,
    ground_pick_active: bool,
    jump_mode: Option<PolicyMode>,
    jump_active: bool,
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
