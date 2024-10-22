"""
Mutlivariable Nonlinear Control
Working on creating animated visuals for the simulation of the robotic arm control system
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import numpy as np
import scipy
import scipy.misc

# external python code files
import ref_gen_2 as ref_gen

# forgetting the control aspect lets get it to plot the linkage system
# closely following this example: https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

# define global parameters
# length measurements are in m
num_int = 1000
r = 0.044
sq_sl = 0.086
tri_sl = 0.094
origin = [0.12, 0]
L1 = 0.095
L2 = 0.095

# generate circle coordinates
# generate reference coordinates
xy = ref_gen.circle_gen(r, origin, num_int)
# [xy,num_int] = ref_gen.square_gen(sq_sl, origin, num_int)
# [xy,num_int] = ref_gen.tri_gen(tri_sl, origin, num_int)
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
Just looking from the animation there are high theta velocities demanded from the linkage arms
Want to optimise reference signal to reduce jerk, and accelerations whilst maintaining high velocity
"""
omega_1 = np.diff(th_1)
omega_2 = np.diff(th_2)

t = range(0,num_int)

# bit of visualisation
# position
plt.figure()
plt.plot(t, th_1, label = "theta 1")
plt.plot(t, th_2, label = "theta 2")
plt.title("Theta")
plt.xlabel("Time (s)")
plt.ylabel("Arm Angle (rad)")
plt.legend()
# velocity
plt.figure()
plt.plot(t[0:-1], omega_1, label = "omega 1")
plt.plot(t[0:-1], omega_2, label = "omega 2")
plt.title("Omega")
plt.xlabel("Time (s)")
plt.ylabel("Arm Angular Velocity (rad/s)")
plt.legend()
# acceleration
plt.figure()
plt.plot(t, th_1, label = "theta 1")
plt.plot(t, th_2, label = "theta 2")
plt.title("Theta")
plt.xlabel("Time (s)")
plt.ylabel("Arm Angle (rad)")
plt.legend()
# jerk
plt.figure()
plt.plot(t, th_1, label = "theta 1")
plt.plot(t, th_2, label = "theta 2")
plt.title("Theta")
plt.xlabel("Time (s)")
plt.ylabel("Arm Angle (rad)")
plt.legend()


plt.show()
