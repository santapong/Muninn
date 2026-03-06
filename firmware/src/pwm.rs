use embassy_rp::pwm::Pwm;

pub struct Motors<'d> {
    _pwm: Pwm<'d>,
}

impl<'d> Motors<'d> {
    pub fn new(pwm: Pwm<'d>) -> Self {
        Self { _pwm: pwm }
    }
    
    pub fn set_throttle(&mut self, _motor_a_pulse: u16, _motor_b_pulse: u16) {
        // Scaffold for setting throttle
    }
}
