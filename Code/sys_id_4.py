"""
Getting system identification reference signals
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import cm
import math
import numpy as np

# external python code files
import ref_gen_5 as ref_gen
import ref_print_0 as ref_print
import hardware_conv_0  as hardware_conv
import ref_fitting_3 as ref_fit
import ref_sig_plot

# define global parameters
# length measurements are in m
num_int = 1000
r = 0.044
sq_sl = 0.086
tri_sl = 0.094
origin_tri = [0.08, -.005]
origin_sq = [0.12, 0]
L1 = 0.095
L2 = 0.095

# generate reference coordinates
# xy = ref_gen.circle_gen(r, origin, num_int)
# [xy_base,num_int] = ref_gen.square_gen(sq_sl, origin_sq, num_int)
# [xy,num_int] = ref_gen.tri_gen(tri_sl, origin, num_int)

# saving array
# refs = np.empty((n_sf,n_cl), dtype=object)

def get_cs_sig(sq_sl, origin_sq, num_int, c_sf, c_l):
        [xy,num_int_cs] = ref_gen.corner_stretch_sq(sq_sl, origin_sq, num_int, c_sf_arr[i], c_l_arr[j])
        # split array for testing
        x_b = xy[:,0]
        y_b = xy[:,1]
        # print(xy)

        # test functions
        ref_gen.test_range(x_b, y_b, L1, L2)
        ref_gen.test_paper(x_b)
        
        [th1_new, th2_new] = ref_gen.get_thetas(xy[:, 0], xy[:, 1], L1, L2)
        th_1_nw = hardware_conv.wrap_ref(th1_new)
        th_2_nw = hardware_conv.wrap_ref(th2_new)
        ref_gen.test_clash(th_2_nw-th_1_nw-180)
        # make time array
        drawtime = 20 # s
        dt = drawtime/num_int
        ref_t_n = np.linspace(0,drawtime, num_int_cs)

        # combining arrays
        # reference = np.column_stack((ref_t, th_1_w, th_2_w))
        ref_cs = np.column_stack((ref_t_n, th_1_nw, th_2_nw))

        enc_per_rot = 131.25*16

        ref_new = hardware_conv.enc_count(ref_cs, enc_per_rot)
        ref_new = hardware_conv.izzy_big_brain(ref_new)
        ref_new = hardware_conv.izzy_big_brain_2(ref_new)

        # for the square and triangle references
        ref_new = hardware_conv.Lizzy_adj(ref_new, 4)
        return ref_new

# want to predict which signals have the lowest max acceleration
n = 100
c_sf_arr = np.linspace(0.4, 0.9, n)
c_l_arr = np.linspace(0.2, 0.4, n)
X, Y = np.meshgrid(c_sf_arr, c_l_arr)

max_ddth = np.empty((n,n))
for j in range(0,n):
    for i in range(0,n):
        # getting acceleration
        ref = get_cs_sig(sq_sl, origin_sq, num_int, c_sf_arr[j], c_l_arr[i])
        
        dth = np.diff(ref, axis=0)
        ddth = np.diff(dth, axis=0)
        # print(type(ddth[:,1]))
        max_ddth[i,j] = np.max(ddth[:,1:2])

# filtering values for better visulisation
# plt_idx = np.where(max_ddth<4)

# Plot the surface
plt.style.use('_mpl-gallery')
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.plot_surface(X, Y, max_ddth, vmin=max_ddth.min() * 2, cmap=cm.Blues)

ax.set(xlabel="Stretch Factor",
       ylabel="Corner Length",
       zlabel="Max acceleration")

plt.show()
          
