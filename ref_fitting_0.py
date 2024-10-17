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

def S_curve(th,max_a,dt):
    # find velocity and acceleration
    om = np.diff(th) #  len(th) -1
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
            brk_idx = np.append(brk_idx, len(th))

    # split into start and end arrays (even = start, odd = end)
    start_idx = brk_idx[::2]
    end_idx = brk_idx[1::2]

    print("start indicies = ",start_idx)
    print("end indicies = ", end_idx)
    
    # Set up initial S_th values
    num_t = np.zeros((len(brk_idx%2))) # number of start end pairs

    # finding t values
    for i in range(len(start_idx)):
        t = np.sqrt((2*(np.abs(th[end_idx[i]]- th[start_idx[i]])/max_a)))
        print(t)
        num_t[i] = int(np.ceil(t/dt))

    # building S_th
    S_th = np.zeros(((len(th)+sum(num_t)),1))
    for i in range(len(start_idx)):
        # fak fak fak i can't fucking figure out how to fucking do this
        
    return S_th