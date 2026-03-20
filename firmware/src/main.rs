#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::I2C0;
use embassy_rp::i2c::{InterruptHandler, Config as I2cConfig, I2c};
use muninn_firmware::mpu6050::Mpu6050;
use muninn_firmware::pwm::{self, QuadMotors};
use muninn_firmware::pid::PidController;
use muninn_firmware::lqg::LqrController;
use muninn_firmware::filter::ComplementaryFilter;
use muninn_firmware::mixer;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

/// Set to `true` to use LQR controller, `false` for PID.
const USE_LQR: bool = false;

/// Motor: 2212 2200KV 6T Brushless Outrunner
/// - At 3S (11.1V): max ~24,420 RPM (no load)
/// - Recommended prop: 5x4.5" or 6x3"
/// - Typical hover throttle for ~500g quad: ~1150-1250us
/// - ESC signal: standard PWM 1000-2000us at 50Hz
const BASE_THROTTLE: f32 = 1180.0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Muninn Firmware v0.3.0 starting...");

    let p = embassy_rp::init(Default::default());

    // --- I2C / IMU Setup ---
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, I2cConfig::default());
    let mut mpu = Mpu6050::new(i2c);

    if let Err(_) = mpu.init().await {
        error!("MPU6050 init failed!");
    }
    info!("MPU6050 initialized.");

    // --- PWM / Motor Setup (4 ESCs on 2 PWM slices) ---
    let pwm_config = pwm::esc_pwm_config();
    let pwm_ch0 = embassy_rp::pwm::Pwm::new_output_ab(
        p.PWM_SLICE0, p.PIN_0, p.PIN_1, pwm_config.clone(),
    );
    let pwm_ch1 = embassy_rp::pwm::Pwm::new_output_ab(
        p.PWM_SLICE1, p.PIN_2, p.PIN_3, pwm_config,
    );
    let mut motors = QuadMotors::new(pwm_ch0, pwm_ch1);

    // Arm ESCs (hold minimum throttle for 2 seconds)
    motors.arm().await;

    // --- Attitude Estimation ---
    let mut attitude = ComplementaryFilter::new(0.98);

    // --- PID Controllers (Roll, Pitch, Yaw) ---
    // Tuned for 2212 2200KV motors with 5" props on ~500g frame.
    // High KV motors are responsive — keep Kp moderate to avoid oscillation.
    // Start with these values, then increase Kp until oscillation, back off 30%.
    //
    //                             Kp    Ki    Kd   i_lim  out_lim
    let mut pid_roll  = PidController::new(3.5,  0.02, 1.8,  40.0,  180.0);
    let mut pid_pitch = PidController::new(3.5,  0.02, 1.8,  40.0,  180.0);
    let mut pid_yaw   = PidController::new(2.5,  0.01, 0.3,  25.0,   80.0);

    // --- LQR Controller (alternative) ---
    let lqr = LqrController::new();

    let dt: f32 = 0.01; // 100Hz control loop

    info!("Control mode: {}", if USE_LQR { "LQR" } else { "PID" });
    info!("Entering main control loop...");

    loop {
        embassy_time::Timer::after_millis(10).await;

        if let Ok(raw) = mpu.read_raw().await {
            let imu = muninn_firmware::data::ImuData::from_raw(&raw);

            // Update attitude estimate (complementary filter)
            attitude.update(&imu, dt);

            let (roll_cmd, pitch_cmd, yaw_cmd) = if USE_LQR {
                // --- LQR Mode ---
                // State error: [roll, pitch, yaw, p, q, r]
                // Setpoint is level hover (all zeros)
                let (gx_rad, gy_rad, gz_rad) = imu.gyro_rad_s();
                let state_error = [
                    attitude.roll,
                    attitude.pitch,
                    attitude.yaw,
                    gx_rad,
                    gy_rad,
                    gz_rad,
                ];
                let u = lqr.update(state_error);
                (u[0], u[1], u[2])
            } else {
                // --- PID Mode ---
                // Setpoint = 0.0 for level hover on all axes
                let roll_out = pid_roll.update(0.0, attitude.roll, dt);
                let pitch_out = pid_pitch.update(0.0, attitude.pitch, dt);
                let yaw_out = pid_yaw.update(0.0, attitude.yaw, dt);
                (roll_out, pitch_out, yaw_out)
            };

            // Mix control outputs into 4 motor commands
            let outputs = mixer::mix(BASE_THROTTLE, roll_cmd, pitch_cmd, yaw_cmd);

            // Send to ESCs
            motors.set_throttles(outputs.fl, outputs.fr, outputs.bl, outputs.br);

            info!(
                "R:{} P:{} Y:{} | FL:{} FR:{} BL:{} BR:{}",
                attitude.roll, attitude.pitch, attitude.yaw,
                outputs.fl, outputs.fr, outputs.bl, outputs.br
            );
        }
    }
}
