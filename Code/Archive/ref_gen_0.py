# for multivariable nonlinear control module
# 2 arm linkage system
# code to find the reference theta values for desired x and y

import math
import numpy as np

def get_thetas(tar_x, tar_y, L_1, L_2):

    # Using cosine rule to find internal angles alpha_1, alpha_2
    phi = np.arctan2(tar_y,tar_x)
    c_sq = np.power(tar_x,2) + np.power(tar_y,2)
    alpha_1 = np.arccos((c_sq + np.power(L_1,2) - np.power(L_2,2))/(2*L_1*np.sqrt(c_sq)))
    alpha_2 = np.arccos((np.power(L_1,2) + np.power(L_2,2) - c_sq)/(2*L_1*L_2))

    # convert to global linkage angles theta_1, theta_2
    theta_1 = phi + alpha_1
    theta_2 = math.pi + phi + alpha_1 + alpha_2

    # convert to degrees
    # theta_1 = math.degrees(theta_1)
    # theta_2 = math.degrees(theta_2)
    return theta_1, theta_2
    
# generating a test circle
def circle_gen(r, origin, num_int):
    circ_theta = np.linspace(0,2*math.pi,num_int)
    circ_xy = np.zeros((num_int, 2))
    for i in range(0, num_int):
        circ_xy[i,0] = r*math.cos(circ_theta[i]) + origin[0]
        circ_xy[i,1] = r*math.sin(circ_theta[i]) + origin[1]

    return circ_xy



