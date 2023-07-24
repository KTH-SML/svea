from copy import deepcopy
import matplotlib.pyplot as plt
import numpy as np
import math

def smooth_path(path, alpha = 0.035, beta = 0.965, tol = 0.000001, smoothing = True):
    # based on  gradient ascent
    # Inputs:
    #   path: the unsmooth path in [x,y] coordinates
    #   alpha: how much the trajectory should be smoothed 
    #   beta: how much the smoothed coordinates should be updated 
    #   tol: tolerance of convergance 
    # Output:
    #   smoothed: the smoothed path

    xs = []; ys = []
    multiplier = 9 # 6
    for i in range(np.shape(path)[0]-1):
        # Generate intermediate points between each pair of waypoints
        path_len = multiplier*math.sqrt( (path[i][0]-path[i + 1][0])**2+(path[i][1]-path[i + 1][1])**2 )
        xs_temp = np.linspace( path[i][0],  path[i+1][0], round(magnitude_order(path_len)*path_len))
        ys_temp = np.linspace( path[i][1],  path[i+1][1], round(magnitude_order(path_len)*path_len))
        xs = xs + xs_temp.tolist()
        ys = ys + ys_temp.tolist()

    path = []
    for i in range(len(xs)):
        path.append([xs[i],ys[i]])

    if smoothing:
        smooth = deepcopy(path)
        length = len(path[0])
        delta = tol
        while delta >= tol:
            delta = 0
            for i in range(1, len(smooth) - 1):
                for j in range(length):
                    x_i = path[i][j]
                    y_i = smooth[i][j]
                    y_prev = smooth[i - 1][j]
                    y_next = smooth[i + 1][j]
                    y_i_old = y_i
                    y_i += alpha * (x_i - y_i) + beta * (y_next + y_prev - (2 * y_i))
                    smooth[i][j] = y_i
                    delta += abs(y_i - y_i_old)

        # Generate yaw target
        yaw_target = generate_yaw_target(smooth)

        # Add yaw target to smooth
        for i in range(len(smooth)):
            smooth[i].append(yaw_target[i])
                
        return smooth
    
    else:
        # Generate yaw target
        yaw_target = generate_yaw_target(path)

        # Add yaw target to smooth
        for i in range(len(path)):
            path[i].append(yaw_target[i])
                
        return path


def generate_yaw_target(path):
    yaw_target = []
    for i in range(len(path)-1):
        yaw_target_temp = math.atan2(path[i+1][1]-path[i][1],path[i+1][0]-path[i][0])

        # Wrap yaw_target_temp to -pi to pi if it exceeds the limits
        if yaw_target_temp > math.pi:
            wraped_yaw_target = yaw_target_temp - 2*math.pi
        elif yaw_target_temp < -math.pi:
            wraped_yaw_target = yaw_target_temp + 2*math.pi
        else:
            wraped_yaw_target = yaw_target_temp

        yaw_target.append(wraped_yaw_target)
    yaw_target.append(yaw_target[-1])
    return yaw_target


def magnitude_order(num):
    ctr = 0
    if num < 1 and num > 0:
        while num < 1:
            num *= 10
            ctr += 1
    return 10**ctr


def plot_path(path,smooth):
    # Plots the original path in black and the smoothed one in red
    x_list = []
    y_list = []
    for x,y in path:
        x_list.append(x)
        y_list.append(y)
    plt.plot(x_list,y_list,c='k')
    x_list = []
    y_list = []
    for x,y in smooth:
        x_list.append(x)
        y_list.append(y)
    plt.plot(x_list,y_list,c='r')
    plt.legend('Original path', 'Smooth path')
    plt.grid()

    plt.show()