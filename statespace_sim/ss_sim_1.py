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

    # def mass_matrix(self):
    #     """ Calculate the mass matrix M(theta). """
    #     M11 = (self.m_a1 + self.m_m1) * (self.L1/2)**2 + (self.m_a2 + self.m_p) * (self.L1**2 + (self.L2/2)**2 + 2 * self.L1 * (self.L2/2) * np.cos(self.th2)) + self.I1 + self.I2
    #     M12 = (self.m_a2 + self.m_p) * ((self.L2/2)**2 + self.L1 * (self.L2/2) * np.cos(self.th2)) + self.I2
    #     M21 = M12
    #     M22 = (self.m_a2 + self.m_p) * (self.L2/2)**2 + self.I2
    #     return np.array([[M11, M12], [M21, M22]])
    
    # def coriolis_matrix(self):
    #     """ Calculate the Coriolis matrix C(theta, omega). """
    #     C11 = - (self.m_a2 + self.m_p) * self.L1 * (self.L2/2) * np.sin(self.th2) * self.dth2
    #     C12 = - (self.m_a2 + self.m_p) * self.L1 * (self.L2/2) * np.sin(self.th2) * (self.dth1 + self.dth2)
    #     C21 = (self.m_a2 + self.m_p) * self.L1 * (self.L2/2) * np.sin(self.th2) * self.dth1
    #     C22 = 0
    #     return np.array([[C11, C12], [C21, C22]])