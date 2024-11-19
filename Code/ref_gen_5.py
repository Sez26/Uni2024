# for multivariable nonlinear control module
# 2 arm linkage system
# code to find the reference theta values for desired x and y

import math
import numpy as np
import sys
import matplotlib.pyplot as plt

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
    theta_1 = np.degrees(theta_1)
    theta_2 = np.degrees(theta_2)

    return theta_1, theta_2

# generating a circle
def circle_gen(r, origin, num_int):
    circ_theta = np.linspace(0,2*math.pi,num_int)
    circ_xy = np.zeros((num_int, 2))
    circ_xy[:,0] = r*np.cos(circ_theta) + origin[0]
    circ_xy[:,1] = r*np.sin(circ_theta) + origin[1]

    return circ_xy

# generating a square
# reference signal starts in the top left, goes clockwise
def square_gen(side_l, origin, num_int):
    ni_side = int(np.floor(num_int/4))

    sq_xy = np.zeros((ni_side*4,2))
    # top hztl side
    sq_xy[0:ni_side, 0] = (origin[0] - side_l/2) + np.linspace(0,side_l,ni_side)
    sq_xy[0:ni_side, 1] = origin[1] + side_l/2
    # right vtcl side
    sq_xy[ni_side:2*ni_side, 0] = origin[0] + side_l/2
    sq_xy[ni_side:2*ni_side, 1] = (origin[1] + side_l/2) - np.linspace(0,side_l,ni_side)
    # bottom hztl side
    sq_xy[2*ni_side:3*ni_side, 0] = (origin[0] + side_l/2) - np.linspace(0,side_l,ni_side)
    sq_xy[2*ni_side:3*ni_side, 1] = origin[1] - side_l/2   
    # left vtcl side
    sq_xy[3*ni_side:4*ni_side, 0] = origin[0] - side_l/2
    sq_xy[3*ni_side:4*ni_side, 1] = (origin[1] - side_l/2) + np.linspace(0,side_l,ni_side)    

    arr_len = len(sq_xy)
    return sq_xy, arr_len

def sq_gen_45(side_l, origin, num_int):
    ni_side = int(np.floor(num_int/4))
    sq_xy = np.zeros((ni_side*4,2))

    sp_x = origin[0]-side_l*np.cos(np.radians(45))
    sp_y = origin[1]
    # rising side 1
    sq_xy[0:ni_side, 0] = sp_x + np.linspace(0,side_l*np.cos(np.radians(45)),ni_side)
    sq_xy[0:ni_side, 1] = sp_y + np.linspace(0,side_l*np.sin(np.radians(45)),ni_side)
    # falling side 2
    sq_xy[ni_side:2*ni_side, 0] = sp_x + side_l*np.cos(np.radians(45)) + np.linspace(0,side_l*np.cos(np.radians(45)),ni_side)
    sq_xy[ni_side:2*ni_side, 1] = sp_y + side_l*np.sin(np.radians(45)) - np.linspace(0,side_l*np.sin(np.radians(45)),ni_side)
    # falling side 3
    sq_xy[2*ni_side:3*ni_side, 0] = sp_x + 2*side_l*np.cos(np.radians(45)) - np.linspace(0,side_l*np.cos(np.radians(45)),ni_side)
    sq_xy[2*ni_side:3*ni_side, 1] = sp_y - np.linspace(0,side_l*np.sin(np.radians(45)),ni_side)
    # rising side 4
    sq_xy[3*ni_side:4*ni_side, 0] = sp_x + side_l*np.cos(np.radians(45)) - np.linspace(0,side_l*np.cos(np.radians(45)),ni_side)
    sq_xy[3*ni_side:4*ni_side, 1] = sp_y - side_l*np.sin(np.radians(45)) + np.linspace(0,side_l*np.sin(np.radians(45)),ni_side)

    arr_len = len(sq_xy)
    return sq_xy, arr_len

def corner_stretch_sq(side_l, origin, num_int, corner_factor, corner_l):
    # corner_factor is the fraction of values allocated to corners rather on the straight
    # corner_length is the fraction of side length that is considered in the corner zone 
    ni_side = int(np.floor(num_int/4))
    sq_xy = np.zeros((ni_side*4,2))

    sp_x = origin[0]-side_l*np.cos(np.radians(45))
    sp_y = origin[1]

    corner_factor = corner_factor/2
    ni_corner = int(ni_side*corner_factor)
    # print(ni_corner)
    ni_side = int(ni_side*(1-2*corner_factor))
    # print(ni_side)

    # rising side 1, corner start
    sq_xy[0:ni_corner, 0] = np.linspace(0,side_l*corner_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[0:ni_corner, 1] = np.linspace(0,side_l*corner_l*np.sin(np.radians(45)),ni_corner)
    # rising side 1, side
    sq_xy[ni_corner:ni_corner+ni_side, 0] = np.linspace(sq_xy[ni_corner-1,0],side_l*(1-corner_l)*np.cos(np.radians(45)),ni_side)
    sq_xy[ni_corner:ni_corner+ni_side, 1] = np.linspace(sq_xy[ni_corner-1,1],side_l*(1-corner_l)*np.sin(np.radians(45)),ni_side)
    # rising side 1, corner end
    # print(np.shape(sq_xy[(ni_corner+ni_side):(2*ni_corner+ni_side), 0]))
    sq_xy[ni_corner+ni_side:2*ni_corner+ni_side, 0] = np.linspace(sq_xy[ni_corner+ni_side-1,0],side_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[ni_corner+ni_side:2*ni_corner+ni_side, 1] = np.linspace(sq_xy[ni_corner+ni_side-1,1],side_l*np.sin(np.radians(45)),ni_corner)

    # falling side 2, corner start
    # print(np.shape(sq_xy[(2*ni_corner+ni_side):(3*ni_corner+ni_side), 0]))
    sq_xy[2*ni_corner+ni_side:3*ni_corner+ni_side, 0] = sq_xy[2*ni_corner+ni_side-1,0] + np.linspace(0,side_l*corner_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[2*ni_corner+ni_side:3*ni_corner+ni_side, 1] = sq_xy[2*ni_corner+ni_side-1,1] - np.linspace(0,side_l*corner_l*np.sin(np.radians(45)),ni_corner)

    sq_xy[3*ni_corner+ni_side:3*ni_corner+2*ni_side, 0] = sq_xy[2*ni_corner+ni_side-1,0] + np.linspace(sq_xy[ni_corner-1,0],side_l*(1-corner_l)*np.cos(np.radians(45)),ni_side)
    sq_xy[3*ni_corner+ni_side:3*ni_corner+2*ni_side, 1] = sq_xy[2*ni_corner+ni_side-1,1] - np.linspace(sq_xy[ni_corner-1,1],side_l*(1-corner_l)*np.sin(np.radians(45)),ni_side)

    # falling side 2, corner end
    sq_xy[3*ni_corner+2*ni_side:4*ni_corner+2*ni_side, 0] = sq_xy[2*ni_corner+ni_side-1,0] + np.linspace(sq_xy[ni_corner+ni_side-1,0],side_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[3*ni_corner+2*ni_side:4*ni_corner+2*ni_side, 1] = sq_xy[2*ni_corner+ni_side-1,1] - np.linspace(sq_xy[ni_corner+ni_side-1,1],side_l*np.sin(np.radians(45)),ni_corner)

    # falling side 3, corner start
    sq_xy[4*ni_corner+2*ni_side:5*ni_corner+2*ni_side, 0] = sq_xy[4*ni_corner+2*ni_side-1,0] - np.linspace(0,side_l*corner_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[4*ni_corner+2*ni_side:5*ni_corner+2*ni_side, 1] = sq_xy[4*ni_corner+2*ni_side-1,1] - np.linspace(0,side_l*corner_l*np.sin(np.radians(45)),ni_corner)

    sq_xy[5*ni_corner+2*ni_side:5*ni_corner+3*ni_side, 0] = sq_xy[4*ni_corner+2*ni_side-1,0] - np.linspace(sq_xy[ni_corner-1,0],side_l*(1-corner_l)*np.cos(np.radians(45)),ni_side)
    sq_xy[5*ni_corner+2*ni_side:5*ni_corner+3*ni_side, 1] = sq_xy[4*ni_corner+2*ni_side-1,1] - np.linspace(sq_xy[ni_corner-1,1],side_l*(1-corner_l)*np.sin(np.radians(45)),ni_side)

    # falling side 3, corner end
    sq_xy[5*ni_corner+3*ni_side:6*ni_corner+3*ni_side, 0] = sq_xy[4*ni_corner+2*ni_side-1,0] - np.linspace(sq_xy[ni_corner+ni_side-1,0],side_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[5*ni_corner+3*ni_side:6*ni_corner+3*ni_side, 1] = sq_xy[4*ni_corner+2*ni_side-1,1] - np.linspace(sq_xy[ni_corner+ni_side-1,1],side_l*np.sin(np.radians(45)),ni_corner)

    # rising side 4, corner start
    sq_xy[6*ni_corner+3*ni_side:7*ni_corner+3*ni_side, 0] = sq_xy[6*ni_corner+3*ni_side-1,0] - np.linspace(0,side_l*corner_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[6*ni_corner+3*ni_side:7*ni_corner+3*ni_side, 1] = sq_xy[6*ni_corner+3*ni_side-1,1] + np.linspace(0,side_l*corner_l*np.sin(np.radians(45)),ni_corner)

    sq_xy[7*ni_corner+3*ni_side:7*ni_corner+4*ni_side, 0] = sq_xy[6*ni_corner+3*ni_side-1,0] - np.linspace(sq_xy[ni_corner-1,0],side_l*(1-corner_l)*np.cos(np.radians(45)),ni_side)
    sq_xy[7*ni_corner+3*ni_side:7*ni_corner+4*ni_side, 1] = sq_xy[6*ni_corner+3*ni_side-1,1] + np.linspace(sq_xy[ni_corner-1,1],side_l*(1-corner_l)*np.sin(np.radians(45)),ni_side)

    # risining side 4, corner end
    sq_xy[7*ni_corner+4*ni_side:8*ni_corner+4*ni_side, 0] = sq_xy[6*ni_corner+3*ni_side-1,0] - np.linspace(sq_xy[ni_corner+ni_side-1,0],side_l*np.cos(np.radians(45)),ni_corner)
    sq_xy[7*ni_corner+4*ni_side:8*ni_corner+4*ni_side, 1] = sq_xy[6*ni_corner+3*ni_side-1,1] + np.linspace(sq_xy[ni_corner+ni_side-1,1],side_l*np.sin(np.radians(45)),ni_corner)
    
    # add offsets for starting point
    sq_xy[:,0] = sq_xy[:,0] + sp_x
    sq_xy[:,1] = sq_xy[:,1] + sp_y

    arr_len = len(sq_xy)
    # print(sq_xy[-10:,:])
    # plt.figure()
    # plt.plot(sq_xy[:,0], sq_xy[:,1], '|-')
    # plt.show()
    return sq_xy, arr_len

# generating a triangle
# start bottom left goes clockwise
def tri_gen(side_l, start_cood, num_int):
    ni_side = int(np.floor(num_int/3))
    tri_xy = np.zeros((3*ni_side,2))
    tri_h = np.sqrt(3)/2 * side_l
    # rising side
    tri_xy[0:ni_side, 0] = start_cood[0] + np.linspace(0,(side_l/2),ni_side)
    tri_xy[0:ni_side, 1] = start_cood[1] + np.linspace(0,tri_h,ni_side)
    # falling side
    tri_xy[ni_side:2*ni_side, 0] = start_cood[0] + side_l/2 + np.linspace(0,(side_l/2),ni_side)
    tri_xy[ni_side:2*ni_side, 1] = start_cood[1] + tri_h - np.linspace(0,tri_h,ni_side)
    # base
    tri_xy[2*ni_side:3*ni_side, 0] = start_cood[0] + side_l - np.linspace(0,side_l,ni_side)
    tri_xy[2*ni_side:3*ni_side, 1] = start_cood[1]

    arr_len = len(tri_xy)
    return tri_xy, arr_len

def sys_id_0(max_angle, drawtime, num_int):
    """
    This reference signal sweeps from 0 to max angle to -max angle for one motor then the other
    """
    swp_int = int(np.floor(num_int/6))

    ref = np.zeros((num_int, 3))
    ref[:,0] = np.linspace(0,drawtime, num_int)

    # theta 1
    ref[0:swp_int,1] = np.linspace(0,max_angle,swp_int)
    ref[swp_int:3*swp_int, 1] = np.linspace(max_angle, -max_angle, 2*swp_int)

    # theta 1
    ref[3*swp_int:4*swp_int,2] = np.linspace(0,max_angle,swp_int)
    ref[4*swp_int:6*swp_int,2] = np.linspace(max_angle, -max_angle, 2*swp_int)

    return ref, 6*swp_int

# test valid range
""" 
conditions for the demand thetas to be invalid:
1) outside of maximum swept area of arms (this should get caught by acos domain validity (returning NaN result))
2) x>0 (edge of paper at armA base)
3) alpha_2 > 0, arms can't clash (in reality this will be a range)
"""

def test_range(tar_x, tar_y, L1, L2):
    # tar_x and tar_y coordinates into polar form (just radius)
    tar_r = np.sqrt(np.power(tar_x, 2) + np.power(tar_y, 2))
    if any(tar_r>(L1+L2)):
        print("Trajectory greater than maximum robot arm swept area. Invalid target positions.")
        sys.exit(1)
    else:
        print("Trajectory within range.")
    return

def test_paper(tar_x):
    if all(tar_x>0):
        print("Trajectory within paper range")
    else:
        print("Trajectory goes off paper (x<0). Invalid target positions.")
        sys.exit(1)
    return

def test_clash(alpha_2):
    if any(alpha_2==0):
        print("During trajectory robot arms clash. Invalid target positions.")
        sys.exit(1)
    else:
        print("Robot arms do not clash during trajectory.")
    return