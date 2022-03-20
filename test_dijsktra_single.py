# -*- coding: utf-8 -*-

import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
from matplotlib import ticker

from map import Map
from searchmethod import Dijkstra


if __name__ == '__main__':
    size = 100
    map = Map(size, size, 1)

    #==============================================
    # Set obstacle vertexs
    #==============================================
    obstacle_points = []

    for i in range(19, 82):
        for j in range(49, 52):
            obstacle_points.append((i,j))
    
    edges = list(np.arange(19, 22)) + list(np.arange(79, 82))
    for i in edges:
        for j in range(49, 72):
            obstacle_points.append((i,j))

    edges = list(np.arange(39, 42)) + list(np.arange(59, 62))
    for i in edges:
        for j in range(29, 72):
            obstacle_points.append((i,j))
    
    edges = list(np.arange(19, 26)) + list(np.arange(35, 42)) \
             + list(np.arange(59, 66)) + list(np.arange(75, 82))
    for i in edges:
        for j in range(69, 72):
            obstacle_points.append((i,j))

    edges = list(np.arange(39, 46)) + list(np.arange(55, 62))
    for i in edges:
        for j in range(29, 32):
            obstacle_points.append((i,j))

    obstacle_points = list(set(obstacle_points))
    map.obstacle(obstacle_points)
    
    dijkstra = Dijkstra(map.obstacle_map)
    finalpath = dijkstra.planning_goal((30,60),(50,40))
    costmap_dict = dijkstra.planning_map((30,60))


    # transer 'costmap_dict' to 'costmap_list'
    costmap_list = [[0 for _ in range(size)]
                       for _ in range(size)]
    costmap_xy = []
    for node in costmap_dict.values():
        costmap_list[node.x][node.y] = node.cost
        costmap_xy.append((node.x, node.y))

    #===========================================
    # Draw the result
    #===========================================
    fig = plt.figure(figsize = (10,10))
    plt.xlim(-1,101)
    plt.ylim(-1,101)
    plt.xticks(np.arange(0,105,5))
    plt.yticks(np.arange(0,105,5))

    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    plt.gcf().set_facecolor(np.ones(3)* 240 / 255)
    plt.grid(True, zorder=0)

    # graph of the expansion obstacles
    for i in range(size):
        for j in range(size):
            if map.obstacle_map[i][j]:
                plt.plot(i,j,'sy', markersize=6)

    # graph of the pure obstacles
    for i in range(len(obstacle_points)):
        plt.plot(obstacle_points[i][0],obstacle_points[i][1],'sk', markersize=6)

    # the generated final path
    path_x, path_y = [],[]
    for i in range(len(finalpath)):
        path_x.append(finalpath[i][0])
        path_y.append(finalpath[i][1])
    plt.plot(path_x, path_y, '-r')
    
    # the heatmap of the mapcost
    maxcost = max(max(costmap_list))
    color = [plt.get_cmap('Blues', 100)(int(costmap_list[ix][iy]/maxcost*100)) for ix,iy in costmap_xy]
    plt.set_cmap(plt.get_cmap('Blues', 100))
    costmap_x = [i for i,j in costmap_xy]
    costmap_y = [j for i,j in costmap_xy]
    
    plt.scatter(costmap_x, costmap_y, marker='s', s=10, c=color)
    #fig.colorbar(im, format=matplotlib.ticker.FuncFormatter(lambda x,pos:int(x*(max_v-min_v)+min_v)))


    plt.title("Grid map simulation")
    plt.show()


    """ set vertexs of the obstacles
    obstacle_vertex_x_1 = [19, 19, 81, 81]
    obstacle_vertex_y_1 = [49, 51, 51, 49]

    obstacle_vertex_x_2 = [39, 39, 35, 35, 41, 41, 45, 45]
    obstacle_vertex_y_2 = [29, 69, 69, 71, 71, 31, 31, 29]

    obstacle_vertex_x_3 = [59, 59, 55, 55, 61, 61, 65, 65]
    obstacle_vertex_y_3 = [71, 31, 31, 29, 29, 69, 69, 71]

    obstacle_vertex_x_4 = [19, 19, 25, 25, 21, 21]
    obstacle_vertex_y_4 = [49, 71, 71, 69, 69, 49]

    obstacle_vertex_x_5 = [79, 79, 75, 75, 81, 81]
    obstacle_vertex_y_5 = [49, 69, 69, 71, 71, 49]

    edge_vertex_x_1 = [-1, -1, 101, 101, 1, 1]
    edge_vertex_y_1 = [-1, 101, 101, 99, 99, -1]
    edge_vertex_x_2 = [-1, 101, 101, 99, 99, -1]
    edge_vertex_y_2 = [-1, -1, 101, 101, 1, 1]


    plt.fill(obstacle_vertex_x_1, obstacle_vertex_y_1, color = "black", zorder=100)
    plt.fill(obstacle_vertex_x_2, obstacle_vertex_y_2, color = "black", zorder=100)
    plt.fill(obstacle_vertex_x_3, obstacle_vertex_y_3, color = "black", zorder=100)
    plt.fill(obstacle_vertex_x_4, obstacle_vertex_y_4, color = "black", zorder=100)
    plt.fill(obstacle_vertex_x_5, obstacle_vertex_y_5, color = "black", zorder=100)
    plt.fill(edge_vertex_x_1, edge_vertex_y_1, color = "black", zorder=100)
    plt.fill(edge_vertex_x_2, edge_vertex_y_2, color = "black", zorder=100)

    """