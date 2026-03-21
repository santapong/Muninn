/// Motor output values in ESC pulse width (microseconds).
#[derive(Debug, Clone, Copy)]
pub struct MotorOutputs {
    pub fl: u16, // Front-Left
    pub fr: u16, // Front-Right
    pub bl: u16, // Back-Left
    pub br: u16, // Back-Right
}

/// ESC pulse width limits
const MIN_PULSE: f32 = 1000.0;
const MAX_PULSE: f32 = 2000.0;

/// Quadcopter X-configuration motor mixer.
///
/// Motor spin directions (viewed from above):
///   FL (CCW)  FR (CW)
///   BL (CW)   BR (CCW)
///
/// Mixing matrix:
///   FL = throttle + roll + pitch - yaw
///   FR = throttle - roll + pitch + yaw
///   BL = throttle + roll - pitch + yaw
///   BR = throttle - roll - pitch - yaw
///
/// `throttle`: base throttle in microseconds (1000-2000 range)
/// `roll`, `pitch`, `yaw`: PID/LQR output corrections
pub fn mix(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
    let fl = clamp_pulse(throttle + roll + pitch - yaw);
    let fr = clamp_pulse(throttle - roll + pitch + yaw);
    let bl = clamp_pulse(throttle + roll - pitch + yaw);
    let br = clamp_pulse(throttle - roll - pitch - yaw);

    MotorOutputs {
        fl: fl as u16,
        fr: fr as u16,
        bl: bl as u16,
        br: br as u16,
    }
}

fn clamp_pulse(val: f32) -> f32 {
    if val < MIN_PULSE {
        MIN_PULSE
    } else if val > MAX_PULSE {
        MAX_PULSE
    } else {
        val
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hover_equal_throttle() {
        let out = mix(1500.0, 0.0, 0.0, 0.0);
        assert_eq!(out.fl, 1500);
        assert_eq!(out.fr, 1500);
        assert_eq!(out.bl, 1500);
        assert_eq!(out.br, 1500);
    }

    #[test]
    fn test_roll_right() {
        // Positive roll → more thrust on left motors, less on right
        let out = mix(1500.0, 50.0, 0.0, 0.0);
        assert_eq!(out.fl, 1550); // left: +roll
        assert_eq!(out.fr, 1450); // right: -roll
        assert_eq!(out.bl, 1550); // left: +roll
        assert_eq!(out.br, 1450); // right: -roll
    }

    #[test]
    fn test_pitch_forward() {
        // Positive pitch → more thrust on front, less on back
        let out = mix(1500.0, 0.0, 50.0, 0.0);
        assert_eq!(out.fl, 1550); // front: +pitch
        assert_eq!(out.fr, 1550); // front: +pitch
        assert_eq!(out.bl, 1450); // back: -pitch
        assert_eq!(out.br, 1450); // back: -pitch
    }

    #[test]
    fn test_yaw_clockwise() {
        // Positive yaw → more thrust on CW motors, less on CCW
        let out = mix(1500.0, 0.0, 0.0, 50.0);
        assert_eq!(out.fl, 1450); // CCW: -yaw
        assert_eq!(out.fr, 1550); // CW:  +yaw
        assert_eq!(out.bl, 1550); // CW:  +yaw
        assert_eq!(out.br, 1450); // CCW: -yaw
    }

    #[test]
    fn test_clamp_output() {
        let out = mix(1900.0, 200.0, 200.0, 0.0);
        assert_eq!(out.fl, 2000); // 2300 clamped to 2000
        assert!(out.fr <= 2000);
        assert!(out.bl >= 1000);
    }
}
