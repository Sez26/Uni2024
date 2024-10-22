"""
This is code to tweak the reference signals so they match the hardware requirements
"""
import numpy as np

def enc_count(ref):
    return

def arm_b_offset(ref):
    ref_th2 = ref[:,2]
    ref_th2 = ref_th2 + 180
    ref_off = np.column_stack((ref[:,[0,1]], ref_th2))
    return ref_off