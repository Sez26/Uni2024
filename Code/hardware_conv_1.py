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

def izzy_big_brain(reference):
    ref = np.copy(reference)
    # make theta_2 datum along axis of arm A
    ref[:,2] = ref[:,1] - ref[:,2]
    return ref

def izzy_big_brain_2(reference):
    ref = np.copy(reference)
    # flipping the datum axis as motors are upside down
    ref[:,1:2] = -ref[:,1:2]
    return ref

def Lizzy_adj(ref, num_sides):
    ref_copy = np.copy(ref)
    # this code changes the start point of the square and triangle reference signals
    # to the middle of a side
    num_int = len(ref_copy)
    ref_spl = np.split(ref_copy, [int(np.floor(num_int/(2*num_sides))),num_int+1])
    # print(np.shape(ref_spl[0]))
    # print(np.shape(ref_spl[1]))
    ref_adj = np.row_stack((ref_spl[1],ref_spl[0]))
    # print(np.shape(ref_adj))
    ref_adj[:,0] = ref[:,0]
    # print(np.shape(ref_adj))
    return ref_adj

def flip_direction(reference):
    ref = np.copy(reference)
    