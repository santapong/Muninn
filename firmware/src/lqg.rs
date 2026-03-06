/// Generates motor mixing commands based on the optimal feedback gain matrix K.
///
/// Under LQR, the control law is:  u = -K * x
/// where:
///   u is the output control effort (torques/forces)
///   K is the pre-calculated gain matrix (from Python math script)
///   x is the state error vector (e.g. difference from hover steady-state)
pub struct LqrController {
    // 1x2 Gain matrix (Scaffold for Pitch only: [K_theta, K_q])
    // These values would be paste-in from the `lqr_calc.py` output
    pub k_matrix: [f32; 2], 
}

impl LqrController {
    pub fn new() -> Self {
        Self {
            // Scaffold values. Replace with real math output later.
            k_matrix: [10.0, 1.0],
        }
    }

    /// Computes the control effort using matrix multiplication: u = -Kx
    pub fn update(&self, state_error: [f32; 2]) -> f32 {
        let mut u = 0.0;
        
        // Dot product of -K and x
        for i in 0..2 {
            u += -self.k_matrix[i] * state_error[i];
        }
        
        u
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lqr_multiplication() {
        let mut lqr = LqrController::new();
        // Manually set K = [10.0, 1.0]
        lqr.k_matrix = [10.0, 1.0];
        
        // State error x = [pitch_err, pitch_rate_err]
        let x = [0.1, 0.5]; // 0.1 rad error, 0.5 rad/s rate error
        
        let u_out = lqr.update(x);
        // u = -(10.0 * 0.1 + 1.0 * 0.5) = -(1.0 + 0.5) = -1.5
        assert_eq!(u_out, -1.5);
    }
}
