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

def S_curve(reference,max_a,dt): #  this can't be done on seperate thetas, because then the thetas would be shifted from eachother
    ref = np.copy(reference)
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

    # find velocity and acceleration
    th = ref[:,1:cols] # remember ranges aren't inclusive of final index
    om = np.diff(th, axis=0) #  len(th) -1
    o_dot = np.diff(om, axis=0) # len(th) -2


    # get index values where acceleration is greater than max_a
    idx = np.abs(o_dot)>max_a
    # print(np.shape(idx))

    # find the first and last index values for each period of saturation
    diff_idx = np.diff(idx, axis=0) # when it != 0 then there is a change of state
    # for some reason this is squishing the arrays

    # print(np.shape(diff_idx))
    brk_idx = np.where(diff_idx>0)
    # print(brk_idx)
    brk_row_idx = brk_idx[0]
    brk_col_idx = brk_idx[1]

    # find start and end pair for each th signal
    th_sig_brk = []
    for i in range(cols-1):
        th_sig_brk.append(brk_row_idx[brk_col_idx==i])
    # print(th_sig_brk)
    # print(len(th_sig_brk[0])/2)
    for i in range(cols-1):
        # check if even (start and end pairs)
        if len(th_sig_brk[i])%2 == 0:
            pass
        elif len(th_sig_brk[i])%2 != 0:
            # check whether it is tail/start
            if idx[0,i]: # if start acceleration is saturated
                th_sig_brk[i] = np.append(0,th_sig_brk[i])
            elif idx[-1,i]: # if end acceleration is saturated
                th_sig_brk[i] = np.append(th_sig_brk[i], len(ref))
        elif len(brk_idx) == 0:
            print("Accelerations do not exceed programmed maximum. Reference is unchanged")
            return ref
    # print(th_sig_brk)
    # print(len(th_sig_brk[0])/2)
    # calculate stretch factor (more like the time required for max_a during saturation)
    num_t = np.array((cols-1, 1), dtype=object)
    for i in range(cols-1):
        num_pair = int(len(th_sig_brk[i])/2)
        num_t[i] = np.zeros((num_pair,1))
        for j in range(int(len(num_t[i]))):
            # num_t[i][j] = int(np.floor(np.sqrt((2*(np.abs(th_sig_brk[i][2*j+1]- th_sig_brk[i][2*j])/max_a)))))
            num_t[i][j] = np.floor(np.sqrt((2*(np.abs(np.diff(th_sig_brk[i])[2*j]/max_a)))))
       
    print(num_t)

    # create substitute arrays
    # Initialize rep_arr as a 1D object array for nesting
    rep_arr = np.empty(cols-1, dtype=object)

    for i in range(cols-1):
        num_pair = int(len(th_sig_brk[i]) / 2)
        print("num_pair:", num_pair)

        # Initialize rep_arr[i] to hold arrays of shape (num_pair, 1)
        rep_arr[i] = np.empty(num_pair, dtype=object)

        for j in range(num_pair):
            print("num_t[i][j][0]:", num_t[i][j][0])

            # Initialize the inner array with zeros
            # print(rep_arr[i][j])

            # this zeros array has to be for all thetas
            rep_arr[i][j] = np.zeros((int(num_t[i][j][0]), cols-1), dtype=float)
            print("rep_arr[i][j] shape: ", np.shape(rep_arr[i][j]))
            for k in range(cols-1):
            # Get index range and fill values based on th_sig_brk and th
                # temp_arr = rep_arr[i][j].copy()
                idx_ra = np.linspace(0, num_t[i][j][0]-1, np.diff(th_sig_brk[i])[2*j], dtype=int)
                fill = th[th_sig_brk[i][2*j]:th_sig_brk[i][2*j+1], k].reshape(-1, 1)
                # Place values from 'fill' into 'rep_arr' using idx_ra indices
                # this is changing for each loop
                rep_arr[i][j][idx_ra, k] = fill.flatten()
                print(np.shape(rep_arr[i][j]))
                # Interpolate to fill in gaps
                rep_arr[i][j][:,k] = np.interp(
                    np.arange(rep_arr[i][j][:,k].shape[0]),  # Full range
                    idx_ra.flatten(),                   # Known index locations
                    fill.flatten()                      # Known values
                ) # Reshape to keep it 2D (num_t[i][j][0], 1)
                print(np.shape(rep_arr[i][j]))

    print("Final rep_arr:", rep_arr[0][0][:,1])



def b_spline_t(reference, num_smpl, dt):
    ref = np.copy(reference)
    # find shape of ref (assume first column is for time series)
    (num_int,cols) = np.shape(ref)
    # number of theta signals (col-1)

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