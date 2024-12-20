import numpy as np

# Physical parameters
m1, m2 = 1.0, 1.0      # Mass of links
l1, l2 = 1.0, 1.0      # Length of links
lc1, lc2 = 0.5, 0.5    # Distance to the center of mass of each link
I1, I2 = 0.2, 0.2      # Moment of inertia of each link
g = 9.81               # Gravitational acceleration

# State vector x = [theta1, theta2, dtheta1, dtheta2]
def mass_matrix(m1, m2, l1, l2, lc1, lc2, I1, I2, theta1, theta2):
    """ Calculate the mass matrix M(theta). """
    M11 = m1 * lc1**2 + m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * np.cos(theta2)) + I1 + I2
    M12 = m2 * (lc2**2 + l1 * lc2 * np.cos(theta2)) + I2
    M21 = M12
    M22 = m2 * lc2**2 + I2
    return np.array([[M11, M12], [M21, M22]])

def coriolis_matrix(m2, l1, lc2, theta2, dtheta1, dtheta2):
    """ Calculate the Coriolis matrix C(theta, omega). """
    C11 = -m2 * l1 * lc2 * np.sin(theta2) * dtheta2
    C12 = -m2 * l1 * lc2 * np.sin(theta2) * (dtheta1 + dtheta2)
    C21 = m2 * l1 * lc2 * np.sin(theta2) * dtheta1
    C22 = 0
    return np.array([[C11, C12], [C21, C22]])

def gravity_vector(m1, m2, l1, lc1, lc2, g, theta1, theta2):
    """ Calculate the gravity vector G(theta). """
    G1 = (m1 * lc1 + m2 * l1) * g * np.cos(theta1) + m2 * lc2 * g * np.cos(theta1 + theta2)
    G2 = m2 * lc2 * g * np.cos(theta1 + theta2)
    return np.array([G1, G2])

def state_space_matrix(m1, m2, l1, l2, lc1, lc2, I1, I2, g, theta1, theta2, dtheta1, dtheta2, tau1, tau2):
    """ Compute the state space matrices A and B. """
    # Compute mass, Coriolis, and gravity terms
    M = mass_matrix(m1, m2, l1, l2, lc1, lc2, I1, I2, theta1, theta2)
    C = coriolis_matrix(m2, l1, lc2, theta2, dtheta1, dtheta2)
    G = gravity_vector(m1, m2, l1, lc1, lc2, g, theta1, theta2)
    
    # State derivatives
    theta_dot = np.array([dtheta1, dtheta2])
    tau = np.array([tau1, tau2])
    
    # Solve for angular accelerations: M * ddtheta = tau - C * dtheta - G
    ddtheta = np.linalg.inv(M) @ (tau - C @ theta_dot - G)
    
    # State-space representation
    A = np.zeros((4, 4))
    A[0, 2] = 1
    A[1, 3] = 1
    
    # Linearized around the current state (approximate A matrix)
    A[2:, 2:] = -np.linalg.inv(M) @ C
    B = np.zeros((4, 2))
    B[2:, :] = np.linalg.inv(M)
    
    return A, B
