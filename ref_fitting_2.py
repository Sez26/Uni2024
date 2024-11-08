"""
We are writing extra functions to alter reference signals to minimise jerk
This code uses the S-curve method, with maximum jerk values

It takes an input of the reference angular position values
Initially assumes equally spaced dt position movements
Assesses acceleration
When maximum acceleration is reached then it maintains maximum acceleration
until target position at end of high jerk period is reached


THIS IS ESSENTIALLY A SATURATION PROBLEM
Essentially anticipating the saturation into the reference signal
Rather than leaving the controller to deal with it
"""

import math
import numpy as np
import sys
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt

def S_curve(ref,max_a,dt): #  this can't be done on seperate thetas, because then the thetas would be shifted from eachother
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

    # find velocity and acceleration
    th = ref[:,1:cols] # remember ranges aren't inclusive of final index
    om = np.diff(th, axis=0) #  len(th) -1
    o_dot = np.diff(om, axis=0) # len(th) -2


    # get index values where acceleration is greater than max_a
    idx = np.abs(o_dot)>max_a
    print(np.shape(idx))

    # find the first and last index values for each period of saturation
    diff_idx = np.diff(idx, axis=0) # when it != 0 then there is a change of state
    # for some reason this is squishing the arrays

    print(np.shape(diff_idx))
    brk_idx = np.where(diff_idx>0)
    print(brk_idx)
    brk_row_idx = brk_idx[0]
    brk_col_idx = brk_idx[1]
    # check if even (start and end pairs)
    if len(brk_row_idx)%2 == 0:
        pass
    elif len(brk_row_idx)%2 != 0:
        # check whether it is tail/start
        if idx[0]: # if start acceleration is saturated
            brk_idx = np.append(0,brk_idx)
        elif idx[-1]: # if end acceleration is saturated
            brk_idx = np.append(brk_idx, len(ref))
    elif len(brk_idx) == 0:
        print("Accelerations do not exceed programmed maximum. Reference is unchanged")
        return ref

    # find start and end pair for each th signal
    th_sig_brk = []
    for i in range(cols-1):
        th_sig_brk.append(brk_row_idx[brk_col_idx==i])

    print(th_sig_brk)

    # # assign maximum acceleration values to all point
    # sat_acc = np.clip(o_dot, -max_a, max_a)   
    # print(np.shape(sat_acc)) 

    # # integrate back
    # sat_o = np.zeros_like(om)
    # sat_o[0,:] = om[0,:]
    # sat_th = np.zeros_like(th)
    # sat_th[0,:] = th[0,:]
    # for i in range(1, len(sat_acc)):
    #     for j in range(cols-1):
    #         sat_o[i,j] = sat_o[i-1,j] + sat_acc[i-1,j] * dt
    # for i in range(1, len(sat_o)):
    #     for j in range(cols-1):
    #         sat_th[i,j] = sat_th[i-1,j] + sat_o[i-1,j] * dt
    


    # # split theta values at break points
    # spl_th = np.split(th,brk_row_idx)
    # print(len(spl_th))

    # # replace arrays
    # # need to make this more robust
    # S_th = np.column_stack((spl_th[0], rep_arrs[0], spl_th[2], rep_arrs[1]))

    # for i in range(len(start_idx)):

    return sat_th


def b_spline_t(reference, num_smpl, dt):
    ref = np.copy(reference)
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

    # find velocity and acceleration
    # om = np.diff(ref, axis=0) #  len(th) -1
    # om[:,0] = ref[:-1, 0]
    # print(np.shape(om))
    # o_dot = np.diff(om, axis=0) # len(th) -2

    # sample base velocity signal at num_smpl
    smpl_ref = ref[::(int(np.floor(num_int/num_smpl)))]
    smple_ref = np.insert(smpl_ref, 0, ref[0,:], axis=0)
    smple_ref = np.insert(smpl_ref, -1, ref[-1,:], axis=0)
    # print(np.shape(smpl_ref))
    spline_order = 5 # quintic spline
    # print(np.shape(new_v))
    ref_new = np.copy(ref)
    # print(np.shape(ref_new))
    # making smooth signal
    smth_t = np.linspace(ref[0,0], ref[-1,0], num_int)

    for i in range(1,cols):
        fit = make_interp_spline(smpl_ref[:,0], smpl_ref[:,i], k=spline_order)
        # print(fit)
        ref_new[:,i] = fit(smth_t[0:num_int])

    return ref_new

    ## this is looking good so I think it is the integrations
    # plt.figure()
    # plt.plot(smpl_ref[:,0], smpl_ref[:,1], 'o', color = 'b')
    # plt.plot(smth_t[0:-1], new_v[:,1], linewidth = 3)
    # plt.plot(smpl_ref[:,0], smpl_ref[:,2], 'o', color = 'r')
    # plt.plot(smth_t[0:-1], new_v[:,2], linewidth = 3)
    # plt.show()

def b_spline_v(reference, num_smpl, dt):
    ref = np.copy(reference)
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

    # find velocity and acceleration
    om = np.diff(ref, axis=0) #  len(th) -1
    om[:,0] = ref[:-1, 0]
    # print(np.shape(om))
    # o_dot = np.diff(om, axis=0) # len(th) -2

    # sample base velocity signal at num_smpl
    smpl_ref = om[::(int(np.floor(num_int/num_smpl)))]
    smple_ref = np.insert(smpl_ref, 0, om[0,:], axis=0)
    smple_ref = np.insert(smpl_ref, -1, om[-1,:], axis=0)
    # print(np.shape(smpl_ref))
    spline_order = 5 # quintic spline
    new_v = np.zeros((num_int-1,3))
    # print(np.shape(new_v))
    ref_new = np.copy(ref)
    # print(np.shape(ref_new))
    # making smooth signal
    smth_t = np.linspace(ref[0,0], ref[-1,0], num_int)

    for i in range(1,cols):
        fit = make_interp_spline(smpl_ref[:,0], smpl_ref[:,i], k=spline_order)
        # print(fit)
        new_v[:,i] = fit(smth_t[0:-1])

    ## this is looking good so I think it is the integrations
    # plt.figure()
    # plt.plot(smpl_ref[:,0], smpl_ref[:,1], 'o', color = 'b')
    # plt.plot(smth_t[0:-1], new_v[:,1], linewidth = 3)
    # plt.plot(smpl_ref[:,0], smpl_ref[:,2], 'o', color = 'r')
    # plt.plot(smth_t[0:-1], new_v[:,2], linewidth = 3)
    # plt.show()

    # print(np.shape(new_v))
    # integrate to get reference values
    # get intial positions
    ref_new[:,0] = smth_t
    # init_pos = ref[0,1:2] # actually redundant as ref_new was made as a duplicate
    for j in range(1,cols):
        for i in range(0,len(ref)-1):
            ref_new[i+1,j] = ref_new[i,j] + new_v[i,j]
    
    # print(np.shape(ref_new))
    return ref_new

def b_spline_a(reference, num_smpl, dt):
    ref = np.copy(reference)
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

    # find velocity and acceleration
    om = np.diff(ref, axis=0) #  len(th) -1
    om[:,0] = ref[:-1, 0]
    # print(np.shape(om))
    o_dot = np.diff(om, axis=0) # len(th) -2
    o_dot[:,0] = ref[:-2, 0]

    # sample base velocity signal at num_smpl
    smpl_ref = o_dot[::(int(np.floor(num_int/num_smpl)))]
    smple_ref = np.insert(smpl_ref, 0, o_dot[0,:], axis=0)
    smple_ref = np.insert(smpl_ref, -1, o_dot[-1,:], axis=0)
    print(np.shape(smpl_ref))
    spline_order = 5 # quintic spline
    new_a = np.zeros((num_int-2,3))
    # print(np.shape(new_v))
    ref_new = np.copy(ref)
    # print(np.shape(ref_new))
    # making smooth signal
    smth_t = np.linspace(ref[0,0], ref[-1,0], num_int)

    for i in range(1,cols):
        fit = make_interp_spline(smpl_ref[:,0], smpl_ref[:,i], k=spline_order)
        # print(fit)
        new_a[:,i] = fit(smth_t[0:-2])

    ## this is looking good so I think it is the integrations
    # plt.figure()
    # plt.plot(smpl_ref[:,0], smpl_ref[:,1], 'o', color = 'b')
    # plt.plot(smth_t[0:-1], new_v[:,1], linewidth = 3)
    # plt.plot(smpl_ref[:,0], smpl_ref[:,2], 'o', color = 'r')
    # plt.plot(smth_t[0:-1], new_v[:,2], linewidth = 3)
    # plt.show()

    # print(np.shape(new_v))
    # integrate to get reference values
    # get intial positions
    ref_new[:,0] = smth_t
    new_v = om
    # init_pos = ref[0,1:2] # actually redundant as ref_new was made as a duplicate
    for j in range(1,cols):
        for i in range(0,len(ref)-2):
            new_v[i+1,j] = new_v[i,j] + new_a[i,j]
            ref_new[i+1,j] = ref_new[i,j] + new_v[i,j]
    
    # print(np.shape(ref_new))
    return ref_new