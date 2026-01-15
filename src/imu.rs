use anyhow::Result;

/// IMU data containing gyroscope and projected gravity
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope data [x, y, z] in rad/s (angular velocity in body frame)
    pub gyro: [f64; 3],
    /// Projected gravity [x, y, z] in m/s² (gravity vector rotated to body frame)
    /// This is computed from orientation quaternion: R^T * [0, 0, -9.81]
    /// NOT raw accelerometer (which includes linear acceleration)
    pub accel: [f64; 3],
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro: [0.0; 3],
            accel: [0.0, 0.0, -9.81], // Default projected gravity (upright robot)
        }
    }
}

/// Rotate a vector by a quaternion
/// quat = [w, x, y, z], vec = [x, y, z]
fn quat_rotate_vec(quat: [f64; 4], vec: [f64; 3]) -> [f64; 3] {
    let [w, qx, qy, qz] = quat;
    let [vx, vy, vz] = vec;

    // Quaternion rotation: v' = q * v * q^-1
    // Optimized formula: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
    let cx = qy * vz - qz * vy;
    let cy = qz * vx - qx * vz;
    let cz = qx * vy - qy * vx;

    let cx2 = cy * qz - cz * qy + w * cx;
    let cy2 = cz * qx - cx * qz + w * cy;
    let cz2 = cx * qy - cy * qx + w * cz;

    [
        vx + 2.0 * cx2,
        vy + 2.0 * cy2,
        vz + 2.0 * cz2,
    ]
}

/// IMU controller for BNO055 sensor
/// Currently a dummy implementation, to be replaced with actual BNO055 driver
pub struct ImuController {
    // Placeholder for future BNO055 device handle
    _dummy: (),
}

impl ImuController {
    /// Create a new IMU controller
    /// TODO: Add actual BNO055 initialization when sensor is connected
    pub fn new() -> Result<Self> {
        Ok(Self { _dummy: () })
    }

    /// Read current IMU data
    /// TODO: Replace with actual BNO055 readings
    pub fn read(&mut self) -> Result<ImuData> {
        // Return default values for now
        // When implementing with real BNO055:
        // 1. Read gyroscope (angular velocity) in rad/s
        // 2. Read orientation quaternion [w, x, y, z]
        // 3. Compute projected gravity:
        //    - World gravity = [0.0, 0.0, -9.81]
        //    - Conjugate quaternion to get inverse rotation: quat_conj = [w, -x, -y, -z]
        //    - Rotate gravity to body frame: projected_gravity = quat_rotate_vec(quat_conj, world_gravity)
        // 4. Return ImuData { gyro, accel: projected_gravity }
        //
        // Note: BNO055 quaternion format may need conversion
        // BNO055 uses [w, x, y, z] format which matches our quat_rotate_vec function
        Ok(ImuData::default())
    }
}

impl Default for ImuController {
    fn default() -> Self {
        Self::new().unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_dummy() {
        let mut imu = ImuController::new().unwrap();
        let data = imu.read().unwrap();

        // Check default projected gravity (upright robot)
        assert_eq!(data.accel[2], -9.81);
    }

    #[test]
    fn test_quat_rotate_identity() {
        // Identity quaternion [1, 0, 0, 0] should not rotate
        let quat = [1.0, 0.0, 0.0, 0.0];
        let vec = [1.0, 2.0, 3.0];
        let result = quat_rotate_vec(quat, vec);

        assert!((result[0] - vec[0]).abs() < 1e-10);
        assert!((result[1] - vec[1]).abs() < 1e-10);
        assert!((result[2] - vec[2]).abs() < 1e-10);
    }

    #[test]
    fn test_projected_gravity_upright() {
        // Upright robot: identity rotation
        let quat = [1.0, 0.0, 0.0, 0.0];
        let world_gravity = [0.0, 0.0, -9.81];
        let projected = quat_rotate_vec(quat, world_gravity);

        // Should match world gravity
        assert!((projected[2] - (-9.81)).abs() < 1e-10);
    }
}
