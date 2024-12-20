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

# def S_curve(ref,max_a,dt): #  this can't be done on seperate thetas, because then the thetas would be shifted from eachother
#     # find shape of ref (assume first column is for time series)
#     (num_int,cols) = np.shape(ref)
#     # number of theta signals (col-1)

#     # find velocity and acceleration
#     th = ref[:,1:cols] # remember ranges aren't inclusive of final index
#     om = np.diff(th, axis=0) #  len(th) -1
#     o_dot = np.diff(om, axis=0) # len(th) -2


#     # get index values where acceleration is greater than max_a
#     idx = np.abs(o_dot)>max_a
#     print(np.shape(idx))

#     # find the first and last index values for each period of saturation
#     diff_idx = np.diff(idx) # when it != 0 then there is a change of state
#     # for some reason this is squishing the arrays

#     print(np.shape(diff_idx))
#     brk_idx = np.where(diff_idx>0)[0]
#     print(np.shape(brk_idx))
#     print(brk_idx)
#     # check if even (start and end pairs)
#     if len(brk_idx)%2 == 0:
#         pass
#     elif len(brk_idx)%2 != 0:
#         # check whether it is tail/start
#         if idx[0]:
#             brk_idx = np.append(0,brk_idx)
#         elif idx[-1]:
#             brk_idx = np.append(brk_idx, len(ref))
#     elif len(brk_idx) == 0:
#         print("Accelerations do not exceed programmed maximum. Reference is unchanged")
#         return ref

#     # split into start and end arrays (even = start, odd = end)
#     start_idx = brk_idx[::2]
#     end_idx = brk_idx[1::2]

#     print("start indicies = ",start_idx)
#     print("end indicies = ", end_idx)

#     # split theta values at break points
#     spl_th = np.split(th,brk_idx)

#     rep_arrs = np.empty((len(start_idx),1)) # number of start end pairs, array of arrays
#     # finding t values and generate the replacement arrays
#     for i in range(len(start_idx)):
#         t = np.sqrt((2*(np.abs(th[end_idx[i]]- th[start_idx[i]])/max_a)))
#         # print(t)
#         num_t = int(np.ceil(t/dt))
#         # generate the array
#         rep_arrs[i] = np.linspace(th[start_idx[i]],th[end_idx[i]],num_t)

#     # replace arrays
#     # need to make this more robust
#     S_th = np.column_stack((spl_th[0], rep_arrs[0], spl_th[2], rep_arrs[1]))

#     # for i in range(len(start_idx)):

#     return S_th

import numpy as np

def S_curve(theta_ref, max_acc, time_step):
    """
    Applies an acceleration saturation to a time series of theta references.
    
    Parameters:
    - theta_ref: A 2D numpy array of shape (N, 2), where N is the number of time steps,
      and 2 corresponds to the two theta references.
    - max_acc: The maximum allowed acceleration.
    - time_step: The time step between each point in the time series.
    
    Returns:
    - adjusted_theta_ref: The adjusted theta references with limited acceleration.
    """
    # Calculate velocities
    velocities = np.diff(theta_ref, axis=0) / time_step
    
    # Calculate accelerations
    accelerations = np.diff(velocities, axis=0) / time_step

    # Apply acceleration saturation
    saturated_acc = np.clip(accelerations, -max_acc, max_acc)
    
    # Integrate the saturated accelerations to get new velocities
    adjusted_velocities = np.zeros_like(velocities)
    adjusted_velocities[0] = velocities[0]  # Initial velocity
    
    for i in range(1, len(saturated_acc)):
        adjusted_velocities[i] = adjusted_velocities[i-1] + saturated_acc[i-1] * time_step

    # Integrate adjusted velocities to get new theta references
    adjusted_theta_ref = np.zeros_like(theta_ref)
    adjusted_theta_ref[0] = theta_ref[0]  # Initial position
    
    for i in range(1, len(adjusted_velocities)):
        adjusted_theta_ref[i] = adjusted_theta_ref[i-1] + adjusted_velocities[i-1] * time_step

    return adjusted_theta_ref

# # Example usage
# theta_ref = np.array([[0, 0], [0.1, 0.1], [0.3, 0.3], [0.7, 0.7], [1.5, 1.5]])  # Sample input
# max_acc = 0.2  # Set maximum allowed acceleration
# time_step = 0.1  # Set time step between points

# adjusted_theta_ref = apply_acceleration_saturation(theta_ref, max_acc, time_step)
# print("Adjusted theta references:\n", adjusted_theta_ref)
