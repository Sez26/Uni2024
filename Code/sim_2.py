"""
Mutlivariable Nonlinear Control
Working on creating animated visuals for the simulation of the robotic arm control system
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import numpy as np

# external python code files
import ref_gen_0 as ref_gen

# forgetting the control aspect lets get it to plot the linkage system
# closely following this example: https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

# define global parameters
# test run
num_int = 1000
r = 1
origin = [2, 0]
L1 = 2
L2 = 2

# generate circle coordinates
xy = ref_gen.circle_gen(r, origin, num_int)
# split array for plotting
x_b = xy[:,0]
y_b = xy[:,1]
# print(xy)

# get theta values

[th_1, th_2] = ref_gen.get_thetas(xy[:, 0], xy[:, 1], L1, L2)
# print(thetas[1:10, :])

x_a = L1*np.cos(th_1)
y_a = L1*np.sin(th_1)

# plotting code (adapted)

# fig = plt.figure(figsize=(5, 4))
# ax = fig.add_subplot(autoscale_on=False, xlim=(-1.5, (L1+L2+0.5)), ylim=(-(L1+L2), (L1+L2)))
# ax.set_aspect('equal')
# ax.grid()

# line, = ax.plot([], [], 'o-', lw=2)
# trace, = ax.plot([], [], '.-', lw=1, ms=2)
# time_template = 'time = %.1fs'
# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def animate(i):
    thisx = [0, x_a[i], x_b[i]]
    thisy = [0, y_a[i], y_b[i]]

    history_x = x_b[:i]
    history_y = y_b[:i]

    line.set_data(thisx, thisy)
    trace.set_data(history_x, history_y)
    time_text.set_text(time_template % (i*dt))
    return line, trace, time_text

# animation variables
dt = 0.1

# ani = animation.FuncAnimation(
#     fig, animate, num_int, interval=dt*num_int, blit=True)
# plt.show()

"""
Adding a controller we are moving into time based simulation now
Before we were basically just generating a reference signal and animating it
Now we are simulating a time based simulation
Therefore we need classes and objects = WOOO exciting

This is our helper reference: https://simonebertonilab.com/pid-controller-python/
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

    """ This class represents a car moving in 1D, subject to a throttle force F, with mass m, 
        aerodynamic drag coefficient b, F_max/F_min forces, and time step T. 
    """

    def __init__(self, m, b, F_max_0, F_max_max, v_max, T):
        self.m = m                      # Mass of the car
        self.b = b                      # Aerodynamic drag coefficient
        self.F_max_0 = F_max_0          # Max force applied to the car by the powertrain at 0 speed
        self.F_max_max = F_max_max      # Max force applied to the car by the powertrain at max speed
        self.v_max = v_max              # Max speed (m/s)
        self.T = T                      # Time step
        self.v = 0                      # Speed of the car

    def Step(self, F):

        """ Update the speed of the car based on the applied force F.
        """
        # Max force applied by the powertrain depends on the speed
        v_to_F_max_x_axis = [0, self.v_max]
        F_max_y_axis = [self.F_max_0, self.F_max_max]

        if self.v < v_to_F_max_x_axis[0]:
            F_max = F_max_y_axis[0]
        elif self.v > v_to_F_max_x_axis[-1]:
            F_max = F_max_y_axis[-1]
        else:
            F_max = np.interp(self.v, v_to_F_max_x_axis, F_max_y_axis)

        # Saturate input force
        if F > F_max:
            F_sat = F_max

        # elif F < 0:
            # F_sat = 0
        else:
            F_sat = F

        # Calculate the derivative dv/dt using the input force and the car's speed and properties
        dv_dt = (F_sat - self.b*self.v*self.v)/self.m

        # Update the speed by integrating the derivative using the time step T
        self.v += dv_dt*self.T
    

omega_1 = np.diff(th_1)
omega_2 = np.diff(th_2)

def main():
    # -------- Configuration --------

    # Simulation parameters

    time_step = 0.1
    end_time = time_step*num_int
    length = num_int

    t = np.zeros(length)
    stpA = np.zeros(length)
    stpB = np.zeros(length)
    vA = np.zeros(length)
    vB = np.zeros(length)
    commandA = np.zeros(length)
    commandB = np.zeros(length)

    # Car parameters

    m = 2140
    b = 0.33
    F_max_0 = 22000
    F_max_max = 1710
    v_max = 1000

    # PID parameters

    Kp = 8000.0
    Ki = 70.0
    Kaw = 1.0
    Kd = 20.0
    T_C = 1.0

    # Initialize PID controller
    ctrlA = PID(Kp, Ki, Kd, Kaw, T_C, time_step, F_max_0, -1000, 30000)
    ctrlB = PID(Kp, Ki, Kd, Kaw, T_C, time_step, F_max_0, -1000, 30000)

    # Initialize car with given parameters
    armA = Arm(m, b, F_max_0, F_max_max, v_max, time_step)
    armB = Arm(m, b, F_max_0, F_max_max, v_max, time_step)

    # Iterate through time steps
    for idx in range(0, length):
        t[idx] = idx*time_step
        # Set setpoint
        stpA[idx] = th_1[idx]
        stpB[idx] = th_2[idx]
        
        # Execute the control loop
        vA[idx] = armA.v
        vB[idx] = armB.v
        ctrlA.Step(vA[idx], stpA[idx])
        ctrlB.Step(vB[idx], stpB[idx])
        commandA[idx] = ctrlA.command_sat
        commandB[idx] = ctrlB.command_sat
        armA.Step(commandA[idx])
        armB.Step(commandB[idx])  

    # Plot speed response

    plt.subplot(2, 1, 1)
    plt.plot(t, vA, label="Response")
    plt.plot(t, stpA, '--', label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.legend()
    plt.grid()

    # Plot command force

    plt.subplot(2, 1, 2)
    plt.plot(t, vB, label="Response")
    plt.plot(t, stpB, '--', label="Setpoint")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.legend()
    plt.grid()

    # Display the plots

    plt.show()

main()        