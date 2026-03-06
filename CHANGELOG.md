# Changelog

All notable changes to this project will be documented in this file.

## [0.2.0] - 2026-03-07
### Added
- Created `firmware/src/pid.rs` tracking discrete-time PID logic with Anti-Windup.
- Attached primitive motor mixer utilizing the PID responses in `main.rs`.
- Created offline Python mathematics environment `math/` utilizing NumPy and `python-control`.
- Generated `math/lqr_calc.py` to map $A$ and $B$ quadcopter states arrays and utilize Riccati formulation to generate optimal Feedback loop $K$.
- Added `firmware/src/lqg.rs` module serving as LQR matrix multiplicator utilizing the copied Python matrix outputs.

## [0.1.0] - 2026-03-06
### Added
- Phase 1: Initial `muninn-firmware` project created using `embassy-rs` for Raspberry Pi Pico W.
- Added Cargo and target configurations (`.cargo/config.toml`, `build.rs`, `memory.x`) for `thumbv6m-none-eabi`.
- Scaled MPU6050 I2C initialization logic in `src/mpu6050.rs`.
- Scaffolded standard PWM ESC signal generation in `src/pwm.rs`.
- Created `ImuData` structure with conversion logic and unit tests in `src/data.rs`.
