import numpy as np
import control as ct


def calculate_lqr_gain():
    """
    Calculates the discrete-time LQR gain matrix K for a hovering quadcopter.
    Full 3-axis model with 6 states and 3 control inputs.

    States (x) - 6 elements:
      [phi,        # Roll angle (rad)
       theta,      # Pitch angle (rad)
       psi,        # Yaw angle (rad)
       p,          # Roll rate (rad/s)
       q,          # Pitch rate (rad/s)
       r]          # Yaw rate (rad/s)

    Inputs (u) - 3 elements:
      [tau_phi,    # Roll torque
       tau_theta,  # Pitch torque
       tau_psi]    # Yaw torque
    """

    # --- Physical Parameters ---
    # Motor: 2212 2200KV 6T Brushless Outrunner x4
    # Frame: ~220mm (estimated from 2212 motor class)
    # AUW:  ~500g (estimated with 3S 1300mAh LiPo)
    # Props: 5x4.5" or 6x3"
    #
    # Moments of inertia (estimated for ~220mm X-frame, ~500g AUW)
    # Measure these with bifilar pendulum for best results!
    I_x = 0.0035  # Moment of inertia around roll axis (kg*m^2)
    I_y = 0.0035  # Moment of inertia around pitch axis (kg*m^2)
    I_z = 0.006   # Moment of inertia around yaw axis (kg*m^2) - typically ~2x roll/pitch
    dt = 0.01     # Control loop timestep (100Hz)

    # --- Continuous-time Linearized State-Space Model ---
    # x_dot = A_cont * x + B_cont * u
    #
    # Linearized around hover (small angle assumption):
    #   phi_dot   = p
    #   theta_dot = q
    #   psi_dot   = r
    #   p_dot     = tau_phi / I_x
    #   q_dot     = tau_theta / I_y
    #   r_dot     = tau_psi / I_z

    A_cont = np.array([
        [0, 0, 0, 1, 0, 0],  # phi_dot = p
        [0, 0, 0, 0, 1, 0],  # theta_dot = q
        [0, 0, 0, 0, 0, 1],  # psi_dot = r
        [0, 0, 0, 0, 0, 0],  # p_dot = tau_phi / I_x
        [0, 0, 0, 0, 0, 0],  # q_dot = tau_theta / I_y
        [0, 0, 0, 0, 0, 0],  # r_dot = tau_psi / I_z
    ], dtype=float)

    B_cont = np.array([
        [0,        0,        0       ],
        [0,        0,        0       ],
        [0,        0,        0       ],
        [1.0/I_x,  0,        0       ],
        [0,        1.0/I_y,  0       ],
        [0,        0,        1.0/I_z ],
    ], dtype=float)

    # C and D matrices (full state observation for LQR)
    C_cont = np.eye(6)
    D_cont = np.zeros((6, 3))

    # Create continuous system
    sys_cont = ct.StateSpace(A_cont, B_cont, C_cont, D_cont)

    # Discretize system (Zero-Order Hold)
    sys_disc = sys_cont.sample(dt)
    A_d = np.array(sys_disc.A)
    B_d = np.array(sys_disc.B)

    # --- LQR Penalty Matrices ---
    # Q penalizes state deviations (6x6 diagonal)
    # Higher values = more aggressive correction of that state
    Q = np.diag([
        10.0,   # Penalize roll angle error
        10.0,   # Penalize pitch angle error
        5.0,    # Penalize yaw angle error (less aggressive)
        1.0,    # Penalize roll rate
        1.0,    # Penalize pitch rate
        0.5,    # Penalize yaw rate
    ])

    # R penalizes actuator effort (3x3 diagonal)
    # Higher values = smoother but slower response
    R = np.diag([
        0.1,    # Roll torque cost
        0.1,    # Pitch torque cost
        0.2,    # Yaw torque cost (more conservative)
    ])

    # Solve Discrete Algebraic Riccati Equation (DARE)
    K, S, E = ct.dlqr(A_d, B_d, Q, R)

    # --- Print Results ---
    print("=" * 60)
    print("Muninn Drone - 3-Axis LQR Gain Calculator")
    print("=" * 60)

    print(f"\nPhysical Parameters:")
    print(f"  I_x = {I_x} kg*m^2 (roll)")
    print(f"  I_y = {I_y} kg*m^2 (pitch)")
    print(f"  I_z = {I_z} kg*m^2 (yaw)")
    print(f"  dt  = {dt} s ({1/dt:.0f} Hz)")

    print(f"\nDiscrete A matrix (6x6):\n{A_d}")
    print(f"\nDiscrete B matrix (6x3):\n{B_d}")

    print(f"\nOptimal Gain Matrix K (3x6):\n{K}")

    print(f"\nClosed-loop eigenvalues:\n{E}")

    # Format for Rust paste-in
    print("\n" + "=" * 60)
    print("Paste this into firmware/src/lqg.rs (k_matrix field):")
    print("=" * 60)
    print("k_matrix: [")
    labels = ["tau_roll ", "tau_pitch", "tau_yaw  "]
    state_labels = ["roll", "pitch", "yaw", "p", "q", "r"]
    for i in range(3):
        row = ", ".join(f"{K[i, j]:.6f}" for j in range(6))
        print(f"    // {labels[i]} gains for [{', '.join(state_labels)}]")
        print(f"    [{row}],")
    print("],")


if __name__ == "__main__":
    calculate_lqr_gain()
