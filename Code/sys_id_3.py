"""
Getting system identification reference signals
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
origin_sq = [0.12, -0.03]
L1 = 0.095
L2 = 0.095

# generate reference coordinates
# xy = ref_gen.circle_gen(r, origin, num_int)
# [xy_base,num_int] = ref_gen.square_gen(sq_sl, origin_sq, num_int)
# [xy,num_int] = ref_gen.tri_gen(tri_sl, origin, num_int)

# optimising for corner stretch factor and corner length
n_sf = 10
n_cl = 3
c_sf_arr = np.linspace(0.4, 0.9, n_sf)
c_l_arr = np.linspace(0.2,0.4,n_cl)

# saving array
refs = np.empty((n_sf,n_cl), dtype=object)

# for loop
for j in range(0,n_cl):
    for i in range(0,n_sf):
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
        refs[i,j] = np.copy(ref_new)

# saving files for loop
save_dir = '/home/sez26/Uni2024/MVNLC/Uni2024_MVNLC/reference_signals/corner_stretch_ref/sq(0.12,-0.03)/'

for j in range(0,n_cl):
    for i in range(0,n_sf):
        filename = f"csf_{np.round(c_sf_arr[i],1)}_cl_{np.round(c_l_arr[j],1)}.h"
        ref_print.print_ref(save_dir,filename, refs[i,j])


# want to predict which signals have the lowest max acceleration
max_ddth = np.empty((n_sf,n_cl))
for j in range(0,n_cl):
    for i in range(0,n_sf):
        # getting acceleration
        ref = np.copy(refs[i,j])
        
        dth = np.diff(ref, axis=0)
        ddth = np.diff(dth, axis=0)
        # print(type(ddth[:,1]))
        max_ddth[i,j] = np.max(ddth[:,1:2])

plt.figure()
plt.plot(max_ddth)
plt.show()