#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

import rospy

from vector2d import Vector2D
from hexcell import HexCell
from hexmap import HexMap
from turtlebot import TurtleBot

'''
## HDCP Algorithm ##
  - this file features the high-level implementation of the proposed HDCP algorithm,
    according to Algorithm 1 in our paper. 
  - "turtlebot.py" handles low-level perception, control and planning, while
    "hexmap.py" provides hex-decomposed map related operations.
  - variables and functions are named explicitly to facilitate understanding. 

## Pipeline ##
  - perception layer
      - read in point cloud data and register on hexmap
  - hex and path planning
      - determine next hex to move according to hexmap
      - find inner/outer tangent points (start and end points)
  - trajectory generation
      - calculate straight-line or circular trajectory
      - generate waypoints according to sampling period (discretization resolution)
  - tracking controller
      - track waypoints according to current robot status
      - feedback linearization is used for handling non-holonomic kinematics
'''

class HexDecompositionCoveragePlanning:
    def __init__(self):
        # read in ros params
        rospy.init_node('hdcp_planner')
        hex_radius = rospy.get_param("/hdcp_planner/hex_radius", 1.0)
        robot_turning_radius = rospy.get_param("/hdcp_planner/robot_turning_radius", 0.5)
        init_x = rospy.get_param("/hdcp_planner/x", 0)
        init_y = rospy.get_param("/hdcp_planner/y", 0)
        data_path = rospy.get_param("/hdcp_planner/data_path")
        exploration_mode = rospy.get_param("/hdcp_planner/exploration_mode", False)
        debugging_mode = rospy.get_param("/hdcp_planner/debugging_mode", False)

        # motion planning parameters
        self.R_hex = float(hex_radius)                   # hex radius equals to the side length
        self.R_traj = float(robot_turning_radius)        # radius of circular trajectory that robot follows
        self.exploration_mode = exploration_mode  # whether to run the variant HDCP-E algorithm

        # initialize hex map and robot interface
        self.hexmap = HexMap(self.R_hex)
        self.robot = TurtleBot(self.hexmap, self.R_traj)    # TODO

        # let's go
        try:
            self.run()
        except rospy.ROSInterruptException as error:
            print(error)
        finally:
            self.visualization(data_path, [init_x, init_y], debugging_mode)


    def run(self):
        init_point = Vector2D(self.R_traj, 0)
        current_hex = HexCell(0, 0, 0)  # in cube coordinate
        current_hex_center = Vector2D(0.0, 0.0) # in cartesian coordiante
        self.robot.initial_circle_trajectory_planning()
        self.hexmap.add_visited(current_hex)
        while not rospy.is_shutdown():
            if not self.hexmap.is_visited(current_hex): # observing mode
                if not self.exploration_mode:
                    self.robot.circle_trajectory_planning(init_point, init_point, current_hex_center)  # full circle
                self.hexmap.add_visited(current_hex)
            self.hexmap.update_obstacles()
            next_hex = self.next_hex(current_hex)
            if self.hexmap.coverage_completed is True:
                # could go back to the origin, but just stop here for now
                self.robot.pub_vel_hp([0, 0])
                break
            start_point, end_point = self.robot.find_tangent_points(current_hex, next_hex, init_point)   # inner and outer
            if abs(init_point - start_point)>0.01: # transitioning mode
                self.robot.circle_trajectory_planning(init_point, start_point, current_hex_center)
            self.robot.turning_clockwise = self.robot.turning_clockwise_buffer
            self.robot.straight_line_trajectory_planning(start_point, end_point) # transitioning mode
            current_hex = next_hex
            current_hex_center = self.hexmap.cube_to_cat(next_hex)
            init_point = end_point


    def next_hex(self, current_hex):
        # find an unexplored hex that has the maximum number of explored neighbors
        candidate = self.hexmap.get_hex_from_neighbor(current_hex)  # eq(1) in paper
        
        # if not empty
        if candidate:
            return candidate

        # all neighbors have been explored
        else:
            target_hex = self.hexmap.get_hex_from_history(current_hex)  # eq(2) in paper
            list_of_hex = self.hexmap.get_path_from_A_star(current_hex, target_hex)
            return list_of_hex[0]  # the closest one


    def visualization(self, directory, center, debugging_mode):
        # plot trajectory once finished
        _, ax = plt.subplots(1)
        ax.set_aspect('equal')

        # map boundary
        X_min = -10 - center[0]
        X_max =  10 - center[0]
        Y_min = -10 - center[1]
        Y_max =  10 - center[1]

        # save and plot hex map
        self.hexmap.save_map(directory)
        self.hexmap.plot_map(ax, debugging_mode=debugging_mode)

        # save and plot trajectory
        self.robot.save_and_plot_trajectory(directory, debugging_mode)

        plt.xlim(X_min - 1, X_max + 1)
        plt.ylim(Y_min - 1, Y_max + 1)
        plt.show()


if __name__ == '__main__':
    whatever = HexDecompositionCoveragePlanning()
