"""
Mutlivariable Nonlinear Control
Working on creating animated visuals for the simulation of the robotic arm control system
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import numpy as np

# external python code files
import ref_gen_2 as ref_gen

# forgetting the control aspect lets get it to plot the linkage system
# closely following this example: https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

# define global parameters
# test run
num_int = 100
r = 1
origin = [2, 0]
L1 = 2
L2 = 2

# generate reference coordinates
# xy = ref_gen.circle_gen(r, origin, num_int)
# [xy,num_int] = ref_gen.square_gen(r, origin, num_int)
[xy,num_int] = ref_gen.tri_gen(r, origin, num_int)
# split array for plotting
x_b = xy[:,0]
y_b = xy[:,1]
# print(xy)

# test functions
ref_gen.test_range(x_b, y_b, L1, L2)
ref_gen.test_paper(x_b)

# get theta values

[th_1, th_2] = ref_gen.get_thetas(xy[:, 0], xy[:, 1], L1, L2)

ref_gen.test_clash(th_2-th_1-math.pi)

x_a = L1*np.cos(th_1)
y_a = L1*np.sin(th_1)

# plotting code (adapted)

fig = plt.figure(figsize=(5, 4))
ax = fig.add_subplot(autoscale_on=False, xlim=(-1.5, (L1+L2+0.5)), ylim=(-(L1+L2), (L1+L2)))
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=2)
trace, = ax.plot([], [], '.-', lw=1, ms=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

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

ani = animation.FuncAnimation(
    fig, animate, num_int, interval=dt*num_int, blit=True)
plt.show()