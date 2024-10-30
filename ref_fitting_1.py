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

def S_curve(ref,max_a,dt): #  this can't be done on seperate thetas, because then the thetas would be shifted from eachother
    # find velocity and acceleration
    om = np.diff(ref[:,1:2]) #  len(th) -1
    o_dot = np.diff(om) # len(th) -2

    # get index values where acceleration is greater than max_a
    idx = np.abs(o_dot)>max_a

    # find the first and last index values for each period of saturation
    diff_idx = np.diff(idx) # when it != 0 then there is a change of state
    print(diff_idx)
    brk_idx = np.where(diff_idx>0)[0]
    # check if even (start and end pairs)
    if len(brk_idx)%2 == 0:
        pass
    elif len(brk_idx)%2 != 0:
        # check whether it is tail/start
        if idx[0]:
            brk_idx = np.append(0,brk_idx)
        elif idx[-1]:
            brk_idx = np.append(brk_idx, len(ref))

    # split into start and end arrays (even = start, odd = end)
    start_idx = brk_idx[::2]
    end_idx = brk_idx[1::2]

    print("start indicies = ",start_idx)
    print("end indicies = ", end_idx)

    # split theta values at break points
    spl_th = np.split(ref,brk_idx)

    rep_arrs = np.empty((len(start_idx),1)) # number of start end pairs, array of arrays
    # finding t values and generate the replacement arrays
    for i in range(len(start_idx)):
        t = np.sqrt((2*(np.abs(th[end_idx[i]]- th[start_idx[i]])/max_a)))
        # print(t)
        # num_t[i] = int(np.ceil(t/dt))
        # generate the array
        rep_arrs[i] = np.linspace(th[start_idx[i]],th[end_idx[i]],t)

    # replace arrays
    # need to make this more robust
    S_th = np.column_stack((spl_th[0], rep_arrs[0], spl_th[2], rep_arrs[1]))

    # for i in range(len(start_idx)):

    return S_th