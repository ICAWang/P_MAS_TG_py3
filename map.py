# -*- coding: utf-8 -*-

import math
import numpy as np
from matplotlib import pyplot as plt
#plt.style.use('ggplot')

class Map:
    """
    Generate a grid map for multi-robot task and motion planning.
    """

    def __init__(self, x_width, y_width, robot_radius, resolution=1):
        """
        Initialize grid map for planning.

        Args:
            x_width (int): the length of the map, [m]
            y_width (int): the width of the map, [m]
            resolution (int): resolution of the grid map, default=1
            robot_radius (int): robot radius, [m]
        """
        self.x_width = x_width
        self.y_width = y_width
        self.robot_radius = robot_radius # expansion coefficient, used to construt configuration space
        self.resolution = resolution
        #self.x = self.x_width / self.resolution
        #self.y = self.y_width / self.resolution

        self.obstacle_map = [[False for _ in range(self.y_width)]
                                    for _ in range(self.x_width)]

    def obstacle(self, obstacle_points):
        """
        Obstacles are constructed as two-dimensional arrays.

        Args:
            obstacle_points (a list of 2-d tuples): contains the vertexs of the obstacles
            # obstcale_x (array-like of int[2]): 
            # obstcale_y (array-like of int[2]): 
        """ 
        
        self.ox, self.oy = [], []
        for i in range(len(obstacle_points)):
            self.ox.append(obstacle_points[i][0])
            self.oy.append(obstacle_points[i][1]) 
        
        self.min_ox = min(self.ox) - 1
        self.min_oy = min(self.oy) - 1
        self.max_ox = max(self.ox) + 2
        self.max_oy = max(self.oy) + 2

        for ix in range(self.min_ox, self.max_ox):
            for iy in range(self.min_oy, self.max_oy):
                for iox, ioy in obstacle_points:
                    distance = math.hypot(iox - ix, ioy - iy)
                    if distance <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break


    def show(self):
        """
        Show the grid map with obstacles.
        """
        pass