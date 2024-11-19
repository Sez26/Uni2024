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
import ref_gen_5 as ref_gen
import hardware_conv_1 as hardware_conv
import ref_fitting_3 as ref_fit
import animation_plot
import ref_sig_plot

# forgetting the control aspect lets get it to plot the linkage system
# closely following this example: https://matplotlib.org/stable/gallery/animation/double_pendulum.html#sphx-glr-gallery-animation-double-pendulum-py

# define global parameters
# length measurements are in m
num_int = 1000
r = 0.044
sq_sl = 0.086
tri_sl = 0.094
origin = [0.12, 0]
origin_tri = [0.08, -.005]
origin_sq = [0.12, 0]
L1 = 0.095
L2 = 0.095

# generate circle coordinates
# generate reference coordinates
# xy = ref_gen.circle_gen(r, origin, num_int)
[xy,num_int] = ref_gen.square_gen(sq_sl, origin_sq, num_int)
# [xy,num_int] = ref_gen.sq_gen_45(sq_sl, origin, num_int)
# [xy,num_int] = ref_gen.tri_gen(tri_sl, origin, num_int)

# [xy,num_int_cs] = ref_gen.corner_stretch_sq(sq_sl, origin_sq, num_int, 0.9, 0.2)
# split array for plotting
x_b = xy[:,0]
y_b = xy[:,1]
# print(xy)

# get theta values

[th_1, th_2] = ref_gen.get_thetas(xy[:,0], xy[:,1], L1, L2)

# adding acceleration saturation

th_1_w = hardware_conv.wrap_ref(th_1)
th_2_w = hardware_conv.wrap_ref(th_2)

drawtime = 20 # s
dt = drawtime/num_int

ref_t = np.linspace(0,drawtime, num_int)

reference = np.column_stack((ref_t, th_1_w, th_2_w))
ref_new = reference
# enc_per_rot = 131.25*16
# ref_new = hardware_conv.enc_count(reference, enc_per_rot)
# reference = hardware_conv.izzy_big_brain(reference)

# add acceleration saturation
# max_acc = 0.0015
# th_asat = ref_fit.S_curve(reference, max_acc, dt)
# print(np.shape(th_asat))
# ref_new = reference

# ref_new = np.column_stack((np.linspace(0,drawtime,len(th_asat)),th_asat))

# ref_new = hardware_conv.Lizzy_adj(reference,4)
ref_flip = hardware_conv.izzy_big_brain_2(ref_new)
# ref_new = ref_fit.b_spline_t(ref_new, 50, dt)
# ref_new = ref_fit.S_curve_a(reference, 0.012)
# ref_new = ref_fit.b_spline_v(reference, 50, dt)
# ref_new = ref_fit.b_spline_a(reference, 500, dt)

# animation
x_a = L1*np.cos(np.radians(ref_new[:,1]))
y_a = L1*np.sin(np.radians(ref_new[:,1]))
x_b = x_a + L2*np.cos(np.radians(ref_new[:,2]))
y_b = y_a + L2*np.sin(np.radians(ref_new[:,2]))

# print(y_b)
animation_plot.arm_animation(L1, L2, x_a, x_b, y_a, y_b, xy, num_int, dt)

x_a = L1*np.cos(np.radians(ref_flip[:,1]))
y_a = L1*np.sin(np.radians(ref_flip[:,1]))
x_b = x_a + L2*np.cos(np.radians(ref_flip[:,2]))
y_b = y_a + L2*np.sin(np.radians(ref_flip[:,2]))
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

ref_sig_plot.ref_plot([reference, ref_new], ["base", "s_curve"])
# ref_sig_plot.ref_plot([reference], ["base"])
