"""
Starting again with the statespace simulation

We want to make classes with for Controller and Plant and Feedback 
"""
import numpy as np
import gen_mat_1 as gen_mat

class Plant:
    def __init__(self, m_a1, m_a2, m_m1, m_p, L1, L2):
        self.m_a1 = m_a1
        self.m_a2 = m_a2
        self.m_m1 = m_m1
        self.m_p = m_p
        self.L1 = L1
        self.L2 = L2

        # cumulative masses
        self.m1 = self.m_a1 + self.m_m1
        self.m2 = self.m_a2 + self.m_p

        # distance to CoM in links (UPDATE THIS EQ)
        self.lc1 = self.L1/2
        self.lc2 = self.L2/2

        # calculate moments of inertia of the arms (uniform bars with point masses on the end)
        self.I1 = (1/3)*m_a1*np.power(L1,2) + m_m1*np.power(L1,2)
        self.I2 = (1/3)*m_a2*np.power(L2,2) + m_p*np.power(L2,2)

        # set state variables
        self.th1 = 0
        self.th2 = 0
        self.dth1 = 0
        self.dth2 = 0
        self.ddth1 = 0
        self.ddth2 = 0

        # torque values (changed via controller)
        self.tau1 = 0
        self.tau2 = 0
        self.g = 9.81

    def get_ssmat(self):
        self.A, self.B = gen_mat.state_space_matrix(self.m1, self.m2, self.L1, self.L2, self.lc1, self.lc2, self.I1, self.I2, self.g, self.th1, self.th2, self.dth1, self.dth2, self.tau1, self.tau2)
    
    # def response(self):
        # self.state = np.array([self.th1 self.th2 self.dth1 self.dth2])

# intialising plant
m_a1 = 0.1 # kg
m_a2 = 0.1 # kg
m_m1 = 0.205 # kg
m_p = 0.02 # kg
L1 = 0.095 # m
L2 = 0.095 # m
Robot = Plant(m_a1, m_a2, m_m1, m_p, L1, L2)
Robot.get_ssmat()
print(Robot.A)

# build controller
class PID_controller:

    def __init__(self, Kp, Ki, Kd, Kaw, T_C, T, max, min, max_rate):
        self.Kp = Kp                # Proportional gain
        self.Ki = Ki                # Integral gain
        self.Kd = Kd                # Derivative gain
        self.Kaw = Kaw              # Anti-windup gain
        self.T_C = T_C              # Time constant for derivative filtering
        self.T = T                  # Time step
        self.max = max              # Maximum command
        self.min = min              # Minimum command
        self.max_rate = max_rate    # Maximum rate of change of the command
        self.integral = 0           # Integral term
        self.err_prev = 0           # Previous error
        self.deriv_prev = 0         # Previous derivative
        self.command_sat_prev = 0   # Previous saturated command
        self.command_prev = 0       # Previous command
        self.command_sat = 0        # Current saturated command
        self.command = 0            # Current command
    
    def Tau_OP(self, error):
        