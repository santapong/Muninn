#[derive(Clone, Copy)]
pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    
    integral: f32,
    previous_error: f32,
    
    // Limits
    integral_limit: f32,
    output_limit: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, output_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            integral_limit,
            output_limit,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt_seconds: f32) -> f32 {
        let error = setpoint - measurement;

        // Proportional
        let p_out = self.kp * error;

        // Integral
        self.integral += error * dt_seconds;
        
        // Anti-windup
        if self.integral > self.integral_limit {
            self.integral = self.integral_limit;
        } else if self.integral < -self.integral_limit {
            self.integral = -self.integral_limit;
        }
        
        let i_out = self.ki * self.integral;

        // Derivative
        let derivative = (error - self.previous_error) / dt_seconds;
        let d_out = self.kd * derivative;

        // Compute Output
        let mut output = p_out + i_out + d_out;

        // Output limits
        if output > self.output_limit {
            output = self.output_limit;
        } else if output < -self.output_limit {
            output = -self.output_limit;
        }

        self.previous_error = error;

        output
    }
    
    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_proportional() {
        let mut pid = PidController::new(2.0, 0.0, 0.0, 100.0, 100.0);
        let out = pid.update(10.0, 5.0, 0.1); // Error = 5, kp = 2
        assert_eq!(out, 10.0);
    }

    #[test]
    fn test_anti_windup() {
        let mut pid = PidController::new(0.0, 1.0, 0.0, 10.0, 100.0); // I-Limit = 10.0
        pid.update(20.0, 0.0, 1.0); // i_out wants to be 20.0
        let out = pid.update(0.0, 0.0, 1.0); // Limits at 10.0
        assert_eq!(out, 10.0); 
    }
}
