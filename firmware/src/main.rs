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
use muninn_firmware::pwm::Motors;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Muninn Firmware starting...");

    let p = embassy_rp::init(Default::default());

    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, I2cConfig::default());
    let mut mpu = Mpu6050::new(i2c);
    
    if let Err(_) = mpu.init().await {
        error!("MPU init failed");
    }

    // PWM scaffolding for Motors 1 and 2 (CH0)
    let pwm_config = embassy_rp::pwm::Config::default();
    let pwm_ch0 = embassy_rp::pwm::Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, pwm_config);
    let mut _motors_front = Motors::new(pwm_ch0);

    // PID Scaffolding (Roll and Pitch)
    use muninn_firmware::pid::PidController;
    let mut pid_roll = PidController::new(1.0, 0.0, 0.0, 10.0, 50.0);
    let mut pid_pitch = PidController::new(1.0, 0.0, 0.0, 10.0, 50.0);
    
    let dt = 0.01; // 100Hz assumed for scaffold

    loop {
        embassy_time::Timer::after_millis(10).await;
        if let Ok(raw) = mpu.read_raw().await {
            let data = muninn_firmware::data::ImuData::from_raw(&raw);
            
            // Note: Scaffold setpoints are 0.0 for level hover
            let roll_output = pid_roll.update(0.0, data.gyro_x, dt);
            let pitch_output = pid_pitch.update(0.0, data.gyro_y, dt);
            
            // Scaffold Motor Mixing
            let base_throttle = 1000.0;
            let m1 = base_throttle + roll_output - pitch_output; // FL
            let m2 = base_throttle - roll_output - pitch_output; // FR
            
            _motors_front.set_throttle(m1 as u16, m2 as u16);
            
            info!("M1: {}, M2: {}", m1, m2);
        }
    }
}
