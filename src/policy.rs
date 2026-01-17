use anyhow::Result;
use crate::motor::NUM_MOTORS;
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
    start_time: Instant,
}

impl Policy {
    /// Create a new dummy policy that generates squatting motion
    pub fn new_dummy() -> Result<Self> {
        Ok(Self {
            mode: PolicyMode::Dummy,
            start_time: Instant::now(),
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
            start_time: Instant::now(),
        })
    }

    /// Run inference on an observation to get actions
    ///
    /// # Arguments
    /// * `observation` - The observation vector (51 dimensions)
    ///
    /// # Returns
    /// Action vector (14 dimensions) for motor position offsets
    pub fn infer(&mut self, observation: &Observation) -> Result<[f32; NUM_MOTORS]> {
        match &mut self.mode {
            PolicyMode::Dummy => {
                // Dummy policy: generate squatting motion
                // Frequency: 0.5 Hz, Amplitude: 0.2 rad (hip_pitch, ankle), 0.4 rad (knee)
                let elapsed = self.start_time.elapsed().as_secs_f32();
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

                // Verify output shape
                let shape_dims = output_shape.as_ref();
                if shape_dims.len() != 2 || shape_dims[1] != NUM_MOTORS as i64 {
                    return Err(anyhow::anyhow!(
                        "Unexpected output shape: {:?}, expected [1, {}]",
                        shape_dims,
                        NUM_MOTORS
                    ));
                }

                // Copy output to actions array
                let mut actions = [0.0f32; NUM_MOTORS];
                for i in 0..NUM_MOTORS {
                    actions[i] = output_data[i];
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
        let action = policy.infer(&obs).unwrap();

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

        // Wait a bit so we're not at t=0
        thread::sleep(Duration::from_millis(100));

        let action = policy.infer(&obs).unwrap();

        // Check that leg joints have some motion (not all zeros)
        let has_motion = action[2] != 0.0 || action[3] != 0.0 || action[4] != 0.0 ||
                        action[11] != 0.0 || action[12] != 0.0 || action[13] != 0.0;
        assert!(has_motion, "Dummy policy should generate squat motion");
    }
}
