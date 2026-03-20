/// Full 3-axis LQR controller for quadcopter attitude stabilization.
///
/// State vector x (6 elements):
///   [roll, pitch, yaw, p (roll rate), q (pitch rate), r (yaw rate)]
///   Angles in radians, rates in rad/s.
///
/// Control output u (3 elements):
///   [tau_roll, tau_pitch, tau_yaw]  (torque commands)
///
/// Control law: u = -K * x
/// where K is a 3x6 gain matrix computed offline via Discrete Algebraic Riccati Equation.
pub struct LqrController {
    /// 3x6 gain matrix K: 3 control outputs x 6 state variables.
    /// Row 0: roll torque gains
    /// Row 1: pitch torque gains
    /// Row 2: yaw torque gains
    pub k_matrix: [[f32; 6]; 3],
}

impl LqrController {
    pub fn new() -> Self {
        Self {
            // Scaffold values. Replace with output from math/lqr_calc.py.
            // Diagonal-dominant structure: each axis primarily responds to its own angle + rate.
            k_matrix: [
                // tau_roll  = -K * [roll, pitch, yaw, p, q, r]
                [10.0, 0.0, 0.0, 2.0, 0.0, 0.0],
                // tau_pitch = -K * [roll, pitch, yaw, p, q, r]
                [0.0, 10.0, 0.0, 0.0, 2.0, 0.0],
                // tau_yaw   = -K * [roll, pitch, yaw, p, q, r]
                [0.0, 0.0, 5.0, 0.0, 0.0, 1.0],
            ],
        }
    }

    /// Computes optimal control effort: u = -K * x_error
    ///
    /// `state_error`: [roll_err, pitch_err, yaw_err, p_err, q_err, r_err]
    /// Returns: [tau_roll, tau_pitch, tau_yaw]
    pub fn update(&self, state_error: [f32; 6]) -> [f32; 3] {
        let mut u = [0.0f32; 3];
        for i in 0..3 {
            for j in 0..6 {
                u[i] += -self.k_matrix[i][j] * state_error[j];
            }
        }
        u
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lqr_zero_state() {
        let lqr = LqrController::new();
        let u = lqr.update([0.0; 6]);
        assert_eq!(u, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_lqr_roll_only() {
        let lqr = LqrController::new();
        // 0.1 rad roll error, 0.5 rad/s roll rate error
        let x = [0.1, 0.0, 0.0, 0.5, 0.0, 0.0];
        let u = lqr.update(x);
        // tau_roll = -(10.0 * 0.1 + 2.0 * 0.5) = -(1.0 + 1.0) = -2.0
        assert_eq!(u[0], -2.0);
        // Other axes should be zero (no cross-coupling in scaffold K)
        assert_eq!(u[1], 0.0);
        assert_eq!(u[2], 0.0);
    }

    #[test]
    fn test_lqr_pitch_only() {
        let lqr = LqrController::new();
        let x = [0.0, 0.2, 0.0, 0.0, 0.3, 0.0];
        let u = lqr.update(x);
        // tau_pitch = -(10.0 * 0.2 + 2.0 * 0.3) = -(2.0 + 0.6) = -2.6
        assert!((u[1] - (-2.6)).abs() < 1e-5);
        assert_eq!(u[0], 0.0);
        assert_eq!(u[2], 0.0);
    }

    #[test]
    fn test_lqr_yaw_only() {
        let lqr = LqrController::new();
        let x = [0.0, 0.0, 0.1, 0.0, 0.0, 0.5];
        let u = lqr.update(x);
        // tau_yaw = -(5.0 * 0.1 + 1.0 * 0.5) = -(0.5 + 0.5) = -1.0
        assert_eq!(u[2], -1.0);
    }

    #[test]
    fn test_lqr_combined() {
        let lqr = LqrController::new();
        let x = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
        let u = lqr.update(x);
        // tau_roll  = -(10*0.1 + 2*0.1) = -1.2
        // tau_pitch = -(10*0.1 + 2*0.1) = -1.2
        // tau_yaw   = -(5*0.1 + 1*0.1)  = -0.6
        assert!((u[0] - (-1.2)).abs() < 1e-5);
        assert!((u[1] - (-1.2)).abs() < 1e-5);
        assert!((u[2] - (-0.6)).abs() < 1e-5);
    }
}
