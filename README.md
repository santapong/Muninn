# Muninn
A custom-built autonomous drone project utilizing a Raspberry Pi Pico W as the primary flight controller. The project follows a phased approach, starting with a baseline PID controller for validation before migrating to an optimal Linear Quadratic Gaussian (LQG) controller (LQR + Kalman Filter) for advanced mathematical flight dynamics.

## Project Status
The Muninn embedded firmware is currently being built in `firmware/` using Rust (`embassy-rs`). The mathematical modeling is being calculated offline using Python in `math/`.

### Firmware Features Built
- Rust Cortex-M firmware scaffolding (`thumbv6m-none-eabi`).
- I2C Driver for MPU6050 and PWM driver for ESCs are scaffolded.
- Basic testcases for IMU data conversion implemented.
- Basic discrete PID controller loop built natively in `firmware/src/pid.rs`.
- Scaffolded discrete state-space array multiplier in firmware `src/lqg.rs` for eventual LQR matrices.

### Offline Modeling Built
- `lqr_calc.py` script constructed to calculate Riccati Equation Matrix K based off physical tensors offline using `numpy` and `python-control`.
