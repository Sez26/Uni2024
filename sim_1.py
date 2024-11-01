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
import hardware_conv_0 as hardware_conv
import ref_fitting_2 as ref_fit
import animation_plot

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

# adding acceleration saturation

th_1_w = hardware_conv.wrap_ref(th_1)
th_2_w = hardware_conv.wrap_ref(th_2)

drawtime = 20 # s
dt = drawtime/num_int

ref_t = np.linspace(0,drawtime, num_int)

reference = np.column_stack((ref_t, th_1_w, th_2_w))
# enc_per_rot = 131.25*16
# ref_new = hardware_conv.enc_count(reference, enc_per_rot)
reference = hardware_conv.izzy_big_brain(reference)

# add acceleration saturation
max_acc = 0.0015
th_asat = ref_fit.S_curve(reference, max_acc, dt)
print(np.shape(th_asat))
# ref_new = reference

ref_new = np.column_stack((np.linspace(0,drawtime,len(th_asat)),th_asat))

# animation
x_a = L1*np.cos(th_asat[:,0])
y_a = L1*np.sin(th_asat[:,0])
x_b = x_a + L2*np.cos(th_asat[:,1])
y_b = y_a + L2*np.sin(th_asat[:,1])
animation_plot.arm_animation(L1, L2, x_a, x_b, y_a, y_b, xy, num_int, dt)

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

om_dot_1 = np.diff(omega_1)
om_dot_2 = np.diff(omega_2)

t = range(0,num_int)

# # bit of visualisation
# # position
# plt.figure()
# plt.plot(t, ref_new[:,1], label = "theta 1")
# plt.plot(t, ref_new[:,2], label = "theta 2")
# plt.title("Theta")
# plt.xlabel("Time (s)")
# plt.ylabel("Arm Angle (counts)")
# plt.legend()
# # velocity
# plt.figure()
# plt.plot(t[0:-1], omega_1, label = "omega 1")
# plt.plot(t[0:-1], omega_2, label = "omega 2")
# plt.title("Omega")
# plt.xlabel("Time (s)")
# plt.ylabel("Arm Angular Velocity (rad/s)")
# plt.legend()
# # # acceleration
# plt.figure()
# plt.plot(t[:-2], om_dot_1, label = "omega dot 1")
# plt.plot(t[:-2], om_dot_2, label = "omega dot 2")
# plt.title("Theta")
# plt.xlabel("Time (s)")
# plt.ylabel("Arm Angle Accelerations (rad/s^2)")
# plt.legend()
# # # jerk
# # plt.figure()
# # plt.plot(t, th_1, label = "theta 1")
# # plt.plot(t, th_2, label = "theta 2")
# # plt.title("Theta")
# # plt.xlabel("Time (s)")
# # plt.ylabel("Arm Angle (rad)")
# # plt.legend()


# plt.show()
