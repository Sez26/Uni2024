import matplotlib.pyplot as plt
import numpy as np

def ref_plot(ref_arr, legend):
    """
    Input is an array of all references in standard format that you want to plot
    input ref names for the legend
    """
    plt.figure()
    for i in range(0,len(ref_arr)):
        # position
        plt.plot(ref_arr[i][::10,0], ref_arr[i][::10,1], '|-', label = "theta 1" + legend[i])
        plt.plot(ref_arr[i][::10,0], ref_arr[i][::10,2], '|-', label = "theta 2" + legend[i])
        plt.title("Theta")
        plt.xlabel("Time (s)")
        plt.ylabel("Arm Angle (deg)")
        plt.legend()

    plt.figure()
    for i in range(0,len(ref_arr)):
        # velocity
        plt.plot(ref_arr[i][:-1,0], np.diff(ref_arr[i][:,1]), label = "omega 1" + legend[i])
        plt.plot(ref_arr[i][:-1,0], np.diff(ref_arr[i][:,2]), label = "omega 2" + legend[i])
        plt.title("Omega")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (deg/s)")
        plt.legend()
    
    plt.figure()
    for i in range(0,len(ref_arr)):
        # velocity
        plt.plot(ref_arr[i][:-2,0], np.diff(np.diff(ref_arr[i][:,1])), label = "omega dot 1" + legend[i])
        plt.plot(ref_arr[i][:-2,0], np.diff(np.diff(ref_arr[i][:,2])), label = "omega dot 2" + legend[i])
        plt.title("Omega dot")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Acceleration (deg/s^2)")
        plt.legend()
    plt.show()