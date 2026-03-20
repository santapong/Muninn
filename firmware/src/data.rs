#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuData {
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub temp: f32,
}

impl ImuData {
    /// Convert raw 16-bit readings into physical units.
    /// Assuming Accel range +/- 2g (16384 LSB/g)
    /// Assuming Gyro range +/- 250 deg/s (131 LSB/deg/s)
    /// Returns gyroscope readings converted from deg/s to rad/s.
    pub fn gyro_rad_s(&self) -> (f32, f32, f32) {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        (
            self.gyro_x * DEG_TO_RAD,
            self.gyro_y * DEG_TO_RAD,
            self.gyro_z * DEG_TO_RAD,
        )
    }

    /// Convert raw 16-bit readings into physical units.
    /// Assuming Accel range +/- 2g (16384 LSB/g)
    /// Assuming Gyro range +/- 250 deg/s (131 LSB/deg/s)
    pub fn from_raw(raw: &[u8; 14]) -> Self {
        let ax = i16::from_be_bytes([raw[0], raw[1]]) as f32 / 16384.0;
        let ay = i16::from_be_bytes([raw[2], raw[3]]) as f32 / 16384.0;
        let az = i16::from_be_bytes([raw[4], raw[5]]) as f32 / 16384.0;
        
        // Temperature logic: (raw_temp / 340.0) + 36.53
        let t = i16::from_be_bytes([raw[6], raw[7]]) as f32 / 340.0 + 36.53;
        
        let gx = i16::from_be_bytes([raw[8], raw[9]]) as f32 / 131.0;
        let gy = i16::from_be_bytes([raw[10], raw[11]]) as f32 / 131.0;
        let gz = i16::from_be_bytes([raw[12], raw[13]]) as f32 / 131.0;

        Self {
            accel_x: ax,
            accel_y: ay,
            accel_z: az,
            gyro_x: gx,
            gyro_y: gy,
            gyro_z: gz,
            temp: t,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_conversion() {
        let raw = [
            0x40, 0x00, // ax = 16384 -> 1.0g
            0x00, 0x00, // ay = 0
            0xC0, 0x00, // az = -16384 -> -1.0g
            0x00, 0x00, // temp -> will result in 36.53
            0x00, 0x83, // gx = 131 -> 1.0 deg/s
            0x00, 0x00, // gy = 0
            0xFF, 0x7D, // gz = -131 -> -1.0 deg/s
        ];
        
        let data = ImuData::from_raw(&raw);
        assert_eq!(data.accel_x, 1.0);
        assert_eq!(data.accel_y, 0.0);
        assert_eq!(data.accel_z, -1.0);
        assert_eq!(data.gyro_x, 1.0);
        assert_eq!(data.gyro_y, 0.0);
        assert_eq!(data.gyro_z, -1.0);
        assert_eq!(data.temp, 36.53);
    }
}
