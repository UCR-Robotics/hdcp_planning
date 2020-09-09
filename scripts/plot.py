#!/usr/bin/env python

import matplotlib.pyplot as plt
from hexmap import HexMap
from turtlebot import TurtleBot

def visualization(directory, center, debugging_mode):
    # plot trajectory once finished
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    # map boundary
    X_min = -10 - center[0]
    X_max =  10 - center[0]
    Y_min = -10 - center[1]
    Y_max =  10 - center[1]

    # load and plot hex map
    hexmap = HexMap(radius=1.0)
    hexmap.plot_map(ax, directory, debugging_mode)

    # load and plot trajectory
    TurtleBot.load_and_plot_trajectory(directory, debugging_mode)

    plt.xlim(X_min - 1, X_max + 1)
    plt.ylim(Y_min - 1, Y_max + 1)
    plt.show()


if __name__ == '__main__':
    directory = "../data/hdcp_e"
    robot_init_coordinate = [-9, -9]
    debugging_mode = True
    visualization(directory, robot_init_coordinate, debugging_mode)
