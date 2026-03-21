use embassy_rp::pwm::{Pwm, Config};

/// Motor: 2212 2200KV 6T Brushless Outrunner DC
/// - 4x motors in X-configuration
/// - Each motor driven by its own ESC (4 separate ESCs)
/// - ESC protocol: Standard PWM 50Hz, 1000-2000us pulse width
///
/// ESC pulse width constants (microseconds mapped to PWM compare values at 1MHz clock)
pub const ESC_MIN: u16 = 1000;  // 1000us - motor off / minimum throttle
pub const ESC_MAX: u16 = 2000;  // 2000us - maximum throttle

/// Creates a PWM config suitable for driving brushless ESCs.
///
/// RP2040 system clock = 125 MHz
/// Divider = 125 → effective PWM clock = 1 MHz (1us resolution)
/// Top = 20000 → period = 20ms → frequency = 50 Hz (standard ESC signal)
pub fn esc_pwm_config() -> Config {
    let mut config = Config::default();
    // divider is stored as fixed-point 8.4: integer_part << 4 | frac_part
    // 125 << 4 = 2000, frac = 0
    config.divider = 125 << 4; // 125.0 integer divider
    config.top = 20000;
    config.compare_a = ESC_MIN;
    config.compare_b = ESC_MIN;
    config
}

/// Manages 4 brushless motors via 2 PWM slices (A/B channels each).
///
/// Pin mapping (X-configuration):
///   SLICE0 channel A → Front-Left  (FL) motor
///   SLICE0 channel B → Front-Right (FR) motor
///   SLICE1 channel A → Back-Left   (BL) motor
///   SLICE1 channel B → Back-Right  (BR) motor
pub struct QuadMotors<'d> {
    slice0: Pwm<'d>,
    slice1: Pwm<'d>,
}

impl<'d> QuadMotors<'d> {
    pub fn new(slice0: Pwm<'d>, slice1: Pwm<'d>) -> Self {
        Self { slice0, slice1 }
    }

    /// Set individual throttle values for all 4 motors.
    /// Values are pulse widths in microseconds (1000 = off, 2000 = full).
    pub fn set_throttles(&mut self, fl: u16, fr: u16, bl: u16, br: u16) {
        let fl = fl.clamp(ESC_MIN, ESC_MAX);
        let fr = fr.clamp(ESC_MIN, ESC_MAX);
        let bl = bl.clamp(ESC_MIN, ESC_MAX);
        let br = br.clamp(ESC_MIN, ESC_MAX);

        let mut config0 = esc_pwm_config();
        config0.compare_a = fl;
        config0.compare_b = fr;
        self.slice0.set_config(&config0);

        let mut config1 = esc_pwm_config();
        config1.compare_a = bl;
        config1.compare_b = br;
        self.slice1.set_config(&config1);
    }

    /// Arms ESCs by holding minimum throttle for 2 seconds.
    /// Must be called before any throttle commands will be accepted by the ESCs.
    pub async fn arm(&mut self) {
        defmt::info!("Arming ESCs...");
        self.set_throttles(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
        embassy_time::Timer::after_secs(2).await;
        defmt::info!("ESCs armed.");
    }

    /// Disarms all motors by setting minimum throttle.
    pub fn disarm(&mut self) {
        self.set_throttles(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
        defmt::info!("Motors disarmed.");
    }
}
