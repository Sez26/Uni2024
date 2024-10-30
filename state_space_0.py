"""
Hello all.
We are trying to do state space things.
Firstly a thank you to these guys at North Western for having a beautifully documented
Example for a 2 link robot

https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-1-lagrangian-formulation-of-dynamics-part-2-of-2/#department 

This should be a helper for controller values
And also should act as the statespace basis for further detailed modelling

To Do List:
1) Function to get torque outputs for theta inputs
"""
import numpy as np

def find_torques(m1, m2, L1, L2, th1, th2, omega1, omega2, om_dot1, om_dot2):
    t1 = om_dot1*(m1*np.power(L1,2) + m2(np.power(L1,2)+2*L1*L2*np.cos(th2)) + np.power(L2,2)) + m2*om_dot2*(L1*L2*np.cos(th2) + np.power(L2, 2)) - m2*L1*L2*np.sin(th2)*(2*omega1*omega2 + np.power(omega2,2))
    t2 = m2*om_dot1*(L1*L2*np.cos(th2) + np.power(L2, 2)) + om_dot2*m2*np.power(L2,2) + m2*L1*L2*np.power(omega1,2)*np.sin(th2)

    return t1, t2

# for a force controlled system (which we have)
# for SISO PID contollers

# T_c1 = t1 (for different values of Kp, Ki, Kd values)
# Get a function which outputs tuned K values for input root positions

def SISO_tune():