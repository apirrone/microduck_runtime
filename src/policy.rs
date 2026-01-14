use anyhow::Result;
use crate::motor::NUM_MOTORS;
use crate::observation::Observation;

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
    // TODO: Add ONNX session when model is ready
    // session: ort::Session,
}

impl Policy {
    /// Create a new dummy policy that always outputs zeros
    pub fn new_dummy() -> Result<Self> {
        Ok(Self {
            mode: PolicyMode::Dummy,
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
    ///         session
    ///     })
    /// }
    /// ```
    #[allow(dead_code)]
    pub fn new_onnx(_model_path: &str) -> Result<Self> {
        Ok(Self {
            mode: PolicyMode::Onnx,
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
                // Dummy policy: always return zero actions
                Ok([0.0; NUM_MOTORS])
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
        // Dummy policy should always return zeros
        assert!(action.iter().all(|&x| x == 0.0));
    }
}
