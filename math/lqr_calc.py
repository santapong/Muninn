import numpy as np
import control as ct


def calculate_lqr_gain():
    """
    Calculates the discrete-time LQR gain matrix K for a hovering quadcopter.
    This is a simplified 1D approximation (e.g., pitch only) for scaffolding.

    States (x):
      [theta,       # Pitch angle (rad)
       q]           # Pitch rate (rad/s)

    Inputs (u):
      [tau_theta]   # Pitch torque
    """

    # --- Physical Parameters Scaffold ---
    # These will be replaced by user's actual drone measurements later
    I_y = 0.01  # Moment of inertia around pitch axis (kg*m^2)
    dt = 0.01  # Control loop timestep (100Hz)

    # Continuous-time Linearized State-Space Model: x_dot = A*x + B*u
    # A = [ 0,   1 ]
    #     [ 0,   0 ]
    A_cont = np.array([[0.0, 1.0], [0.0, 0.0]])

    # B = [ 0   ]
    #     [ 1/I ]
    B_cont = np.array([[0.0], [1.0 / I_y]])

    # Create continuous system
    sys_cont = ct.StateSpace(A_cont, B_cont, np.eye(2), np.zeros((2, 1)))

    # Discretize system (Zero-Order Hold)
    sys_disc = sys_cont.sample(dt)
    A_d = sys_disc.A
    B_d = sys_disc.B

    # --- LQR Penalty Matrices ---
    # Q penalizes state deviations (tune these to prioritize angle vs rate)
    Q = np.array(
        [
            [10.0, 0.0],  # Strongly penalize pitch angle error
            [0.0, 1.0],  # Weakly penalize pitch rate error
        ]
    )

    # R penalizes actuator effort (tune this to save battery or limit aggressiveness)
    R = np.array([[0.1]])

    # Solve Discrete Algebraic Riccati Equation (DARE) and get optimal Gain K
    # K matrix will be size (num_inputs x num_states) -> 1x2 in this scaffold
    K, S, E = ct.dlqr(A_d, B_d, Q, R)

    print("--- LQR Setup Complete ---")
    print(f"Discrete A matrix:\n{A_d}")
    print(f"Discrete B matrix:\n{B_d}")
    print(f"\nCalculated Optimal Gain Matrix K:\n{K}")
    print("\nPaste this K matrix into your Rust firmware (src/lqg.rs):")
    print(f"let K = [[{K[0, 0]:.4f}, {K[0, 1]:.4f}]];")


if __name__ == "__main__":
    print("Muninn Drone - Scaffold LQR Math Calculator\n")
    calculate_lqr_gain()
