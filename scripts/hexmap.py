from math import pi, sqrt, atan2, cos, sin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from vector2d import Vector2D
from hexcell import HexCell


class HexMap:
    def __init__(self, radius=1.0):
        self.coverage_completed = False
        self.radius = radius
        self.visited_list = list()
        self.obstacle_list = list()
        self.neighbor_ordering = [HexCell(0,-1, 1), HexCell(1, -1, 0), HexCell(1, 0, -1), #  S, SE, NE
                                  HexCell(0, 1,-1), HexCell(-1, 1, 0), HexCell(-1, 0, 1)] #  N, NW, SW


    def is_visited(self, hexcell):
        return hexcell in self.visited_list


    def is_obstacle(self, hexcell):
        return hexcell in self.obstacle_list


    def is_explored(self, hexcell):
        return hexcell in self.visited_list or hexcell in self.obstacle_list


    def neighbors(self, hexcell):    # hexcell addition is defined in the HexCell class
        return [hexcell + i for i in self.neighbor_ordering]


    def visited_neighbors(self, hexcell):
        return [i for i in self.neighbors(hexcell) if i in self.visited_list]


    def obstacle_neighbors(self, hexcell):
        return [i for i in self.neighbors(hexcell) if i in self.obstacle_list]


    def explored_neighbors(self, hexcell):
        return [i for i in self.neighbors(hexcell) if i in self.visited_list or i in self.obstacle_list]


    def unexplored_neighbors(self, hexcell):
        return [i for i in self.neighbors(hexcell) if i not in self.visited_list and i not in self.obstacle_list]


    def number_of_explored_neighbors(self, hexcell):  # TODO: each hex should have its own property 
        return len(self.explored_neighbors(hexcell))  # (visitable or not) defined in HexCell class


    def add_visited(self, hexcell):
        self.visited_list.append(hexcell)


    def add_obstacle(self, hexcell):
        self.obstacle_list.append(hexcell)


    def update_obstacles(self):
        self.obstacle_list = [i for i in self.obstacle_list if i not in self.visited_list]


    def hex_distance(self, hex1, hex2):
        return abs(hex1-hex2)


    def range_hex(self, current_hex, h=1):  # h is the searching range; h=1 is the same as self.neighbors
        hex_list = list()
        for a in range(-h, h+1):
            for b in range(max(-h, -a-h), min(h, -a+h)+1):
                candidate = HexCell(current_hex[0]+a, current_hex[1]+b, current_hex[2]-a-b)
                if self.hex_distance(candidate, current_hex) == h:
                    hex_list.append(candidate)
        return hex_list


    def cat_to_cube(self, cat):
        # convert from cartesian coordinate to cube coodinate 
        x = 2 * cat.x / (3*self.radius)
        y = (-cat.x + sqrt(3)*cat.y) / (3*self.radius)
        z = (-cat.x - sqrt(3)*cat.y) / (3*self.radius)
        cube = [round(x), round(y), round(z)]
        diff = [abs(cube[0]-x), abs(cube[1]-y), abs(cube[2]-z)]

        if diff[0] > diff[1] and diff[0] > diff[2]:
            cube[0] = - cube[1] - cube[2]
        elif diff[1] > diff[2]:
            cube[1] = - cube[0] - cube[2]
        else:
            cube[2] = - cube[0] - cube[1]
        return HexCell(cube[0], cube[1], cube[2])


    def cube_to_cat(self, cube):
        # convert from cube coordinate to cartesian coodinate
        # project cube y and z onto cat y, where cube coordiante has a constraint x + y + z = 0
        return Vector2D(cube.x * 1.5 * self.radius, (cube.y-cube.z) * sqrt(3)/2 * self.radius)


    def get_hex_from_neighbor(self, hexcell):
        # find an unexplored hex that has the maximum number of explored neighbors
        candidate = self.unexplored_neighbors(hexcell)
        number = [self.number_of_explored_neighbors(i) for i in candidate]
        if number: # not empty
            return candidate[number.index(max(number))]
        else:
            return []


    def get_hex_from_history(self, hexcell):
        # find the first available hex whose neighbors have not been fully explored
        for idx, candidate in reversed(list(enumerate(self.visited_list))):
            num = self.number_of_explored_neighbors(candidate)
            if num < 6:
                return candidate
        # idx and candidate keep the last value after for loop
        if idx == 0 and num == 6:  # meaning all neighbors of the origin have been explored
            self.coverage_completed = True
            return candidate   # should be (0,0,0)
        else:
            raise ValueError("get_hex is not working properly")


    def get_path_from_A_star(self, start, goal):
        open_list = []
        open_list.append((0, start))
        parent = {}           # parent is not closed_list
        parent[start] = None  # we skipped closed_list in this implementation
        past_cost = {}
        past_cost[start] = 0
        if goal in self.obstacle_list:
            raise ValueError("the goal is not reachable in A*")
        # expand open list
        while open_list:
            open_list.sort()
            current = open_list.pop(0)[1]
            if current == goal:
                break
            for candidate in self.neighbors(current):
                if candidate in self.obstacle_list:
                    continue
                new_cost = past_cost[current] + 1 # constant cost = 1
                if candidate not in past_cost or new_cost < past_cost[candidate]:
                    past_cost[candidate] = new_cost
                    heuristic = self.hex_distance(candidate, goal)
                    open_list.append((new_cost + heuristic, candidate))
                    parent[candidate] = current
        # construct the path backward
        path = []
        while current != start:
            path.append(current)
            current = parent[current]
        path.reverse()
        print("A* path: " + str(start) + " " + str(path))
        return path


    def save_map(self, directory):
        if not isinstance(directory, str):
            raise TypeError("please specify the directory using string type")

        obstacle = np.array([list(i.cube) for i in self.obstacle_list])
        np.savetxt(directory + "/obstacle.csv", obstacle, fmt='%f', delimiter=',')

        visited = np.array([list(i.cube) for i in self.visited_list])
        np.savetxt(directory + '/visited.csv', visited, fmt='%f', delimiter=',')


    def plot_map(self, ax, directory=None, debugging_mode=False):
        if directory:
            if not isinstance(directory, str):
                raise TypeError("please specify the directory using string type")
            obstacle_list = np.loadtxt(directory + "/obstacle.csv", delimiter=',')
            visited_list = np.loadtxt(directory + "/visited.csv", delimiter=',')
        else:
            obstacle_list = self.obstacle_list
            visited_list = self.visited_list

        for obstacle in obstacle_list:
            obstacle = HexCell(obstacle[0], obstacle[1], obstacle[2])
            self.plot_hexagon(ax, self.cube_to_cat(obstacle), 'red', ' ')
        
        for (idx, visited) in enumerate(visited_list, 1):
            visited = HexCell(visited[0], visited[1], visited[2])
            if debugging_mode:
                self.plot_hexagon(ax, self.cube_to_cat(visited), 'green', str(idx))
            else:
                self.plot_hexagon(ax, self.cube_to_cat(visited), 'white', ' ')


    def plot_hexagon(self, ax, center, color, label):
        # color in lower case words
        xy = tuple(center)
        hexagon = mpatches.RegularPolygon(xy, numVertices=6, radius=self.radius, 
                            orientation=np.radians(30), 
                            facecolor=color, alpha=0.2, edgecolor='k')
        ax.add_patch(hexagon)
        ax.text(center[0], center[1], label, ha='center', va='center', size=10)


