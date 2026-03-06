# Project Muninn: Autonomous Pico W Drone

## Overview
A custom-built autonomous drone project utilizing a Raspberry Pi Pico W as the primary flight controller. The project follows a phased approach, starting with a baseline PID controller for validation before migrating to an optimal Linear Quadratic Gaussian (LQG) controller (LQR + Kalman Filter) for advanced mathematical flight dynamics.

## Hardware Architecture
* **Microcontroller:** Raspberry Pi Pico W (RP2040)
* **IMU Sensor:** MPU6050 (6-DOF Accelerometer & Gyroscope) via I2C
* **Actuators:** 4x Brushless Motors + 4-in-1 ESC (PWM driven)
* **Power:** 2S/3S LiPo Battery + 5V/3.3V BEC

## Software Stack & Infrastructure
* **Embedded Flight Controller (On-Drone):** Rust (utilizing the `embassy-rs` async framework). Chosen for memory safety, deterministic execution for real-time matrix math, and as a core learning objective.
* **Mathematical Modeling (Offline):** Python (`scipy.signal`, `python-control`). Used to build the state-space model ($A, B$ matrices) and calculate the optimal LQR gain matrix ($K$) offline.
* **Ground Control Station (GCS):** * **Backend:** Python with Litestar to handle high-speed UDP telemetry packets from the Pico W.
    * **Frontend:** TypeScript/JavaScript for a reactive telemetry dashboard.
* **Hosting:** Dockerized GCS containers running locally on the Kali Linux PC (32GB RAM) or Raspberry Pi 5 (16GB RAM) for low-latency network communication.
* **Project Management:** ClickUp for tracking hardware procurement, algorithmic research, and sprint milestones.

---

## Execution Phases

### Phase 1: Hardware & Sensor Validation
* [ ] Flash Rust `embassy-rs` environment to the Pico W.
* [ ] Implement I2C driver to read raw data from the MPU6050.
* [ ] Implement PWM driver to control ESCs and spin motors (props off).

### Phase 2: Telemetry & GCS Setup
* [ ] Scaffold the TypeScript/Litestar Ground Control Station.
* [ ] Deploy GCS via Docker to the local Linux/Pi 5 infrastructure.
* [ ] Establish UDP Wi-Fi link between Pico W and GCS.
* [ ] Stream and visualize live IMU data at 100Hz+.

### Phase 3: PID Baseline (Proof of Flight)
* [ ] Implement a Complementary Filter in Rust for basic pitch/roll estimation.
* [ ] Write a standard PID control loop.
* [ ] Secure the drone, attach propellers, and tune PID for a stable hover.

### Phase 4: Physics Modeling & LQG Preparation
* [ ] Measure physical drone parameters (mass, moment of inertia, motor thrust coefficients).
* [ ] Write Python scripts to define the state-space model: $\dot{x} = Ax + Bu$.
* [ ] Calculate optimal LQR gain matrix ($K$) and determine $Q$ and $R$ penalty matrices.

### Phase 5: The LQG Upgrade
* [ ] Upgrade the Rust sensor fusion from a Complementary Filter to a Kalman Filter.
* [ ] Replace the PID loop with LQR matrix multiplication: $u = -Kx$.
* [ ] Hardcode the Python-calculated $K$ matrix into the Rust firmware.
* [ ] Flight test and iteratively tune the offline $Q$ and $R$ matrices.