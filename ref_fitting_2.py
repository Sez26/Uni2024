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

def Lizzy_adj(ref, num_sides):
    # this code changes the start point of the square and triangle reference signals
    # to the middle of a side
    num_int = len(ref)
    ref_spl = np.split(ref, [int(np.floor(num_int/(2*num_sides))),num_int])
    # print(np.shape(ref_spl[0]))
    # print(np.shape(ref_spl[1]))
    ref_adj = np.row_stack((ref_spl[1],ref_spl[0]))
    # print(np.shape(ref_adj))
    return ref_adj


# def b_spline(ref, num_smpl):


# # Sampled data (velocity values) - replace this with your own data
# time_samples = np.linspace(0, 10, num=20)  # Sampled time points
# velocity_samples = np.array([0, 0.5, 1.5, 3.5, 5.5, 7, 6, 5, 4, 3, 2.5, 3, 4, 5, 5.5, 5.8, 6, 6.2, 6.3, 6.5])  # Sampled velocity values

# # Generate a smoother time vector for the fitted spline
# time_smooth = np.linspace(time_samples[0], time_samples[-1], 200)

# # Fit a B-spline to the sampled data
# spline_order = 3  # Cubic spline
# spline_fit = make_interp_spline(time_samples, velocity_samples, k=spline_order)

# # Evaluate the B-spline for the smoother time vector
# velocity_smooth = spline_fit(time_smooth)
