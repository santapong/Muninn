use crate::data::ImuData;

/// Complementary filter for estimating roll, pitch, and yaw angles
/// by fusing accelerometer and gyroscope data.
///
/// Roll and pitch use accelerometer correction to prevent gyro drift.
/// Yaw uses gyro-only integration (no magnetometer available).
pub struct ComplementaryFilter {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    alpha: f32,
}

impl ComplementaryFilter {
    /// Create a new filter with the given alpha coefficient.
    /// Alpha = 0.98 is typical: 98% gyro trust, 2% accelerometer correction.
    pub fn new(alpha: f32) -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            alpha,
        }
    }

    /// Update attitude estimate with new IMU data.
    ///
    /// `dt` is the time step in seconds (e.g., 0.01 for 100Hz).
    /// Gyro data from ImuData is in deg/s and gets converted to rad/s internally.
    pub fn update(&mut self, imu: &ImuData, dt: f32) {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

        let gx = imu.gyro_x * DEG_TO_RAD;
        let gy = imu.gyro_y * DEG_TO_RAD;
        let gz = imu.gyro_z * DEG_TO_RAD;

        // Accelerometer-based angle estimates
        let accel_roll = libm::atan2f(imu.accel_y, imu.accel_z);
        let accel_pitch = libm::atan2f(
            -imu.accel_x,
            libm::sqrtf(imu.accel_y * imu.accel_y + imu.accel_z * imu.accel_z),
        );

        // Complementary filter: blend gyro integration with accelerometer correction
        self.roll = self.alpha * (self.roll + gx * dt) + (1.0 - self.alpha) * accel_roll;
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1.0 - self.alpha) * accel_pitch;

        // Yaw: gyro-only (will drift without magnetometer)
        self.yaw += gz * dt;
    }

    pub fn reset(&mut self) {
        self.roll = 0.0;
        self.pitch = 0.0;
        self.yaw = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_initial_state() {
        let filter = ComplementaryFilter::new(0.98);
        assert_eq!(filter.roll, 0.0);
        assert_eq!(filter.pitch, 0.0);
        assert_eq!(filter.yaw, 0.0);
    }

    #[test]
    fn test_filter_level_hover() {
        let mut filter = ComplementaryFilter::new(0.98);
        // Simulates level hover: accel_z = 1g, everything else zero
        let imu = ImuData {
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 1.0,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            temp: 25.0,
        };
        for _ in 0..100 {
            filter.update(&imu, 0.01);
        }
        // Should converge to near-zero angles
        assert!(filter.roll.abs() < 0.01);
        assert!(filter.pitch.abs() < 0.01);
        assert!(filter.yaw.abs() < 0.001);
    }

    #[test]
    fn test_filter_tilted() {
        let mut filter = ComplementaryFilter::new(0.98);
        // Simulates ~45 degree roll: accel_y = accel_z = 0.707
        let imu = ImuData {
            accel_x: 0.0,
            accel_y: 0.707,
            accel_z: 0.707,
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            temp: 25.0,
        };
        // Run for several seconds to converge
        for _ in 0..500 {
            filter.update(&imu, 0.01);
        }
        // Should converge to ~0.785 rad (45 degrees)
        let expected = core::f32::consts::PI / 4.0;
        assert!((filter.roll - expected).abs() < 0.05);
    }
}
