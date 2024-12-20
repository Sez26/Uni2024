import numpy as np
import gen_mat_0 as gen_mat

# Example usage
theta1, theta2 = np.pi / 4, np.pi / 4   # Link angles (radians)
dtheta1, dtheta2 = 0.0, 0.0             # Angular velocities (rad/s)
tau1, tau2 = 0.0, 0.0                   # Torques (N*m)

A, B = gen_mat.state_space_matrix(theta1, theta2, dtheta1, dtheta2, tau1, tau2)
print("A matrix:\n", A)
print("B matrix:\n", B)

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Physical parameters (repeated for clarity)
m1, m2 = 1.0, 1.0      # Mass of links
l1, l2 = 1.0, 1.0      # Length of links
lc1, lc2 = 0.5, 0.5    # Distance to the center of mass of each link
I1, I2 = 0.2, 0.2      # Moment of inertia of each link
g = 9.81               # Gravitational acceleration

# PID controller parameters for each joint
Kp1, Ki1, Kd1 = 10.0, 1.0, 0.5  # PID gains for joint 1
Kp2, Ki2, Kd2 = 10.0, 1.0, 0.5  # PID gains for joint 2

# Initial state and reference
theta1_ref = np.pi / 3           # Desired theta1
theta2_ref = np.pi / 6           # Desired theta2
initial_state = [0, 0, 0, 0]     # Initial angles and angular velocities

# Define PID control errors
error_integral = np.array([0.0, 0.0])  # Integral of errors for both joints
previous_error = np.array([0.0, 0.0])  # Previous error (for derivative term)

# Helper functions
def calculate_torque(theta, dtheta, theta_ref):
    """Compute the PID control torque for each joint."""
    global error_integral, previous_error
    
    # Error between desired and actual angles
    error = np.array([theta_ref[0] - theta[0], theta_ref[1] - theta[1]])
    
    # Integral of error
    error_integral += error * dt
    
    # Derivative of error
    derror = (error - previous_error) / dt
    
    # PID control torques
    tau1 = Kp1 * error[0] + Ki1 * error_integral[0] + Kd1 * derror[0]
    tau2 = Kp2 * error[1] + Ki2 * error_integral[1] + Kd2 * derror[1]
    
    # Update previous error
    previous_error = error
    
    return np.array([tau1, tau2])

# Dynamics for integration
def robot_dynamics(state, t, theta_ref):
    """ Compute the state derivatives given current state and reference. """
    theta1, theta2, dtheta1, dtheta2 = state
    
    # Calculate torques from PID control
    tau = calculate_torque([theta1, theta2], [dtheta1, dtheta2], theta_ref)
    
    # Calculate state-space matrices
    M = gen_mat.mass_matrix(theta1, theta2)
    C = gen_mat.coriolis_matrix(theta1, theta2, dtheta1, dtheta2)
    G = gen_mat.gravity_vector(theta1, theta2)
    
    # Solve for angular accelerations
    ddtheta = np.linalg.inv(M) @ (tau - C @ np.array([dtheta1, dtheta2]) - G)
    
    return [dtheta1, dtheta2, ddtheta[0], ddtheta[1]]

# Simulation parameters
dt = 0.01  # Time step
t = np.arange(0, 5, dt)  # Time array
theta_ref = [theta1_ref, theta2_ref]

# Run simulation
state = odeint(robot_dynamics, initial_state, t, args=(theta_ref,))

# Plot results
theta1, theta2 = state[:, 0], state[:, 1]
plt.plot(t, theta1, label='Theta 1')
plt.plot(t, theta2, label='Theta 2')
plt.axhline(theta1_ref, color='r', linestyle='--', label='Theta 1 Reference')
plt.axhline(theta2_ref, color='g', linestyle='--', label='Theta 2 Reference')
plt.xlabel('Time [s]')
plt.ylabel('Joint Angles [rad]')
plt.legend()
plt.title("2-Link Robot Arm PID Control")
plt.show()
