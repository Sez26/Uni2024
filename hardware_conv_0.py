"""
This is code to tweak the reference signals so they match the hardware requirements
"""
import numpy as np

def enc_count(ref, enc_per_rot):
    ref[:,[1,2]] = ref[:,[1,2]] / (360/enc_per_rot)
    ref_new = ref
    return ref_new

def wrap_ref(th):
    # convert theta range from 0 - 360 to -180 - 180
    # check if over 180
    # true then -360
    # otherwise return unchanged theta value
    wrap_th = np.where(th>180,(th-360),th)
    return wrap_th

def izzy_big_brain(ref):
    # make theta_2 datum along axis of arm A
    ref[:,2] = ref[:,1] - ref[:,2]
    return ref

def izzy_big_brain_2(ref):
    # make theta_2 datum along axis of arm A
    ref[:,2] = -ref[:,2]
    return ref