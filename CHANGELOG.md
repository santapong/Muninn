# Changelog

All notable changes to this project will be documented in this file.

## [0.3.0] - 2026-03-20
### Added
- 4-motor brushless ESC support via `QuadMotors` in `firmware/src/pwm.rs` with proper 50Hz PWM signal generation (1000-2000us pulse), ESC arming/disarming sequences.
- Complementary filter for attitude estimation in `firmware/src/filter.rs`, fusing accelerometer and gyroscope data for roll/pitch angles with gyro-only yaw integration.
- X-configuration quadcopter motor mixer in `firmware/src/mixer.rs` with FL/FR/BL/BR mixing matrix and output clamping.
- 3-axis PID control (roll, pitch, yaw) with tunable gains in the main control loop.
- Full 6-state LQR controller (3x6 K matrix) in `firmware/src/lqg.rs` supporting [roll, pitch, yaw, p, q, r] state vector with 3-axis torque output.
- Compile-time PID/LQR controller switch via `USE_LQR` constant in `main.rs`.
- `gyro_rad_s()` helper method on `ImuData` for deg/s to rad/s conversion.
- `libm` dependency for `no_std` math functions (atan2f, sqrtf).
- Feature-gated hardware dependencies for host-target unit testing (`--no-default-features`).
- Expanded `math/lqr_calc.py` to full 6-state 3-axis model with Ix/Iy/Iz moments of inertia and 3x3 R cost matrix.
- 16 unit tests covering PID, LQR, complementary filter, motor mixer, and IMU data conversion.

### Changed
- Upgraded PWM driver from 2-motor scaffold to 4-motor ESC driver using 2 PWM slices (4 GPIO pins).
- Main control loop now integrates: IMU read -> complementary filter -> PID/LQR -> mixer -> 4-motor output.
- LQR expanded from 1x2 pitch-only to 3x6 full 3-axis controller.

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
