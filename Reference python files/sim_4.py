"""
Mutlivariable Nonlinear Control
Working on creating animated visuals for the simulation of the robotic arm control system
Shitty PID implemented
Add some better test functions
Convert to state space representation of the systems
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import numpy as np

# external python code files
import ref_gen_1 as ref_gen
import animation_plot

# forgetting the control aspect lets get it to plot the linkage system
# closely following this example: https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

# define global parameters
# test run
num_int = 1000
r = 0.44
origin = [0.6, 0]
L1 = 0.96
L2 = 0.96

# generate circle coordinates
xy = ref_gen.circle_gen(r, origin, num_int)
# split array for plotting
x_b = xy[:,0]
y_b = xy[:,1]
# print(xy)

# get theta values

[th_1, th_2] = ref_gen.get_thetas(xy[:, 0], xy[:, 1], L1, L2)
# print(thetas[1:10, :])

# x_a = L1*np.cos(th_1)
# y_a = L1*np.sin(th_1)

"""
Adding a controller we are moving into time based simulation now
Before we were basically just generating a reference signal and animating it
Now we are simulating a time based simulation
Therefore we need classes and objects = WOOO exciting

This is our helper reference: https://simonebertonilab.com/pid-controller-python/

Adapting this code to reflect the mechanics of the motors
PWM control
"""

class PID:

    """ This class implements a PID controller.
    """

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

    def Step(self, measurement, setpoint):
        """ Execute a step of the PID controller.

        Inputs:
            measurement: current measurement of the process variable
            setpoint: desired value of the process variable
        """

        # Calculate error
        err = setpoint - measurement

        # Update integral term with anti-windup
        self.integral += self.Ki*err*self.T + self.Kaw*(self.command_sat_prev - self.command_prev)*self.T
        
        # Calculate filtered derivative
        deriv_filt = (err - self.err_prev + self.T_C*self.deriv_prev)/(self.T + self.T_C)
        self.err_prev = err
        self.deriv_prev = deriv_filt

        # Calculate command using PID equation
        self.command = self.Kp*err + self.integral + self.Kd*deriv_filt

        # Store previous command
        self.command_prev = self.command

        # Saturate command
        if self.command > self.max:
            self.command_sat = self.max
        elif self.command < self.min:
            self.command_sat = self.min
        else:
            self.command_sat = self.command

        # Apply rate limiter
        if self.command_sat > self.command_sat_prev + self.max_rate*self.T:
            self.command_sat = self.command_sat_prev + self.max_rate*self.T
        elif self.command_sat < self.command_sat_prev - self.max_rate*self.T:
            self.command_sat = self.command_sat_prev - self.max_rate*self.T

        # Store previous saturated command
        self.command_sat_prev = self.command_sat

class Arm:

    """ This class represents a robot arm rotating. It is subject to a driving moment at the base.
    Arm A also has a compound moment from the actuator of Arm B.
    The rotational inertia is modelled as a 'barbell mass'
    See I_2 in this reference: https://phys.libretexts.org/Bookshelves/University_Physics/University_Physics_(OpenStax)/Book%3A_University_Physics_I_-_Mechanics_Sound_Oscillations_and_Waves_(OpenStax)/10%3A_Fixed-Axis_Rotation__Introduction/10.06%3A_Calculating_Moments_of_Inertia
    J = mL^2
    """

    def __init__(self, m, L, T_max, T, ang_acc, omega, theta):
        self.m = m                      # Mass of end effector
        self.L = L                      # Length of arm
        self.T_max = T_max              # Max torque applied to the arm by origin motor
        self.T = T                      # Time step
        self.ang_acc = ang_acc          # angular acceleration
        self.omega = omega              # angular velocity
        self.theta = theta              # arm angle (to horizontal)

        # calculate rotational inertia
        self.J = m*np.power(L,2)

    def Step(self, T_0, T_1):
        """ Update the angle of the arm based on applied torques (from both origin motor and end effector).
        """

        # Saturate origin motor toque
        if T_0 > self.T_max:
            T_0_sat = self.T_max
        else:
            T_0_sat = T_0

        # Saturate end effector toque
        if T_1 > self.T_max:
            T_1_sat = self.T_max
        else:
            T_1_sat = T_1

        # calculate the resultant angular acceleration of the 
        res_mom = T_0_sat-T_1_sat
        self.ang_acc = res_mom/self.J

        # Update the arm angular velocity by integrating the acceleration using the time step T
        self.omega += self.ang_acc*self.T
        # Update the arm position by integrating the velocity using the time step T
        self.theta += self.omega*self.T
    

def main():
    # -------- Configuration --------

    # Simulation parameters

    time_step = 0.01
    end_time = time_step*num_int
    length = num_int

    t = np.zeros(length)
    stpA = np.zeros(length)
    stpB = np.zeros(length)
    th_A = np.zeros(length)
    th_B = np.zeros(length)
    commandA = np.zeros(length)
    commandB = np.zeros(length)

    # ArmA parameters

    m_a = 200e-3 # kg. Arm A end effector is arm B origin motor
    L_a = 95e-3 # m
    T_max_a = 45e-2 # kgm
    ang_acc_a = 0
    omega_a = 0
    theta_a = 0

    # ArmB parameters

    m_b = 200e-3 # kg. Arm A end effector is arm B origin motor
    L_b = 95e-3 # m
    T_max_b = 45e-2 # kgm
    ang_acc_b = 0
    omega_b = 0
    theta_b = 0

    # PID parameters A

    Kp_a = 5.0
    Ki_a = 10.0
    Kaw_a = 10.0
    Kd_a = 50.0
    T_C_a = 10.0

    # PID parameters B

    Kp_b = 5.0
    Ki_b = 10.0
    Kaw_b = 10.0
    Kd_b = 50.0
    T_C_b = 10.0

    # Initialize PID controller
    ctrlA = PID(Kp_a, Ki_a, Kd_a, Kaw_a, T_C_a, time_step, T_max_a, -1000, 30000)
    ctrlB = PID(Kp_b, Ki_b, Kd_b, Kaw_b, T_C_b, time_step, T_max_b, -1000, 30000)

    # Initialize car with given parameters
    armA = Arm(m_a, L_a, T_max_a, time_step, ang_acc_a, omega_a, theta_a)
    armB = Arm(m_b, L_b, T_max_b, time_step, ang_acc_b, omega_b, theta_b)

    # Iterate through time steps
    for idx in range(0, length):
        t[idx] = idx*time_step
        # Set setpoint
        stpA[idx] = th_1[idx]
        stpB[idx] = th_2[idx]
        
        # Execute the control loop
        th_A[idx] = armA.theta
        th_B[idx] = armB.theta
        ctrlA.Step(th_A[idx], stpA[idx])
        ctrlB.Step(th_B[idx], stpB[idx])
        commandA[idx] = ctrlA.command_sat
        commandB[idx] = ctrlB.command_sat
        armA.Step(commandA[idx], 0)
        armB.Step(commandB[idx], 0)   

    # Plot speed response

    plt.subplot(2, 1, 1)
    plt.plot(t, th_A, label="Response")
    plt.plot(t, stpA, '--', label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle Arm A [rad]")
    plt.legend()
    plt.grid()

    # Plot command force

    plt.subplot(2, 1, 2)
    plt.plot(t, th_B, label="Response")
    plt.plot(t, stpB, '--', label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle Arm B [rad]")
    plt.legend()
    plt.grid()

    # plt.show()

    # animation
    x_a = L1*np.cos(th_A)
    y_a = L1*np.sin(th_A)
    x_b = x_a + L2*np.cos(th_B)
    y_b = y_a + L2*np.sin(th_B)
    animation_plot.arm_animation(L1, L2, x_a, x_b, y_a, y_b, xy, num_int, time_step)

main()        