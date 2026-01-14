use anyhow::Result;
use crate::motor::NUM_MOTORS;
use crate::observation::Observation;
use std::time::Instant;

/// Policy mode
pub enum PolicyMode {
    /// Dummy policy that always outputs zeros (for testing)
    Dummy,
    /// ONNX Runtime policy (not yet implemented)
    #[allow(dead_code)]
    Onnx,
}

/// Policy inference engine using ONNX Runtime
pub struct Policy {
    mode: PolicyMode,
    start_time: Instant,
    // TODO: Add ONNX session when model is ready
    // session: ort::Session,
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
    ///
    /// # TODO
    /// Uncomment and implement when you have an ONNX model:
    /// ```ignore
    /// pub fn new_onnx(model_path: &str) -> Result<Self> {
    ///     ort::init().commit()?;
    ///     let session = ort::Session::builder()?
    ///         .with_optimization_level(ort::GraphOptimizationLevel::Level3)?
    ///         .with_intra_threads(2)? // Limit threads for Raspberry Pi Zero 2W
    ///         .commit_from_file(model_path)?;
    ///     Ok(Self {
    ///         mode: PolicyMode::Onnx,
    ///         start_time: Instant::now(),
    ///         session
    ///     })
    /// }
    /// ```
    #[allow(dead_code)]
    pub fn new_onnx(_model_path: &str) -> Result<Self> {
        Ok(Self {
            mode: PolicyMode::Onnx,
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
    pub fn infer(&self, _observation: &Observation) -> Result<[f32; NUM_MOTORS]> {
        match self.mode {
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
            PolicyMode::Onnx => {
                // TODO: Implement actual ONNX inference:
                // let input = ort::inputs![observation.as_slice()]?;
                // let outputs = self.session.run(input)?;
                // let output: ort::SessionOutputs = outputs;
                // let action = output[0].try_extract_tensor::<f32>()?;
                //
                // let mut result = [0.0f32; NUM_MOTORS];
                // result.copy_from_slice(&action.view().as_slice().unwrap()[..NUM_MOTORS]);
                // Ok(result)
                Ok([0.0; NUM_MOTORS])
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dummy_policy() {
        let policy = Policy::new_dummy().unwrap();
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

        let policy = Policy::new_dummy().unwrap();
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
