# -*- coding: utf-8 -*-

from cmath import inf
import math
import time
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse, Circle
from matplotlib import colors
from matplotlib import ticker

from map import Map
from searchmethod import Dijkstra

def if_graphconnect(G):
    """
    #Warshell algorithm to judge whether the graph is connected.
    Core: find the transitive closure of a graph (it can be directed or undirected)

    Input: 
        G: the adjacency matrix G of the graph with n vertices
        radius: the connected radius
    Args:
        R: the reachability matrix

    Output: True or False

    """
    dim = np.shape(G)[0]  # dimension of the graph

    G = np.mat(G)
    R = np.mat(np.zeros((dim, dim)))

    for i in range(dim):
        R += G ** i
    
    if R.all() > 0:
        return True

    return False


if __name__ == '__main__':

    map_size = 100
    agent_radius = 1
    comm_radius = 5
    map = Map(map_size, map_size, agent_radius)

    #==============================================
    # Set obstacle vertexs
    #==============================================
    obstacle_points = []

    edges = list(np.arange(19, 26)) + list(np.arange(35, 46)) \
            + list(np.arange(55, 66)) + list(np.arange(75, 82))
    for i in edges:
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

    #==============================================
    # Dijkstra discrete planning
    #==============================================    
    dijkstra = Dijkstra(map.obstacle_map)
    start_points = [(30,60), (40,10), 
                    (75,65), (30,55), 
                    (70,40), (60,10)]
    agent_num = int(len(start_points)/2)
    time_start = time.process_time()

    costmap_dict_multi, costmap_list_multi, costmap_xy_multi = dijkstra.planning_map_multiagent(start_points)
    comm_pos_set, agent_distance = dijkstra.min_max_distance(start_points, costmap_list_multi)
    print("This process 1 executes for %.4f s" %((time.process_time()-time_start)))
    #finalpath = dijkstra.path_generate_multi(start_points, comm_pos_set)
    region_radius = (agent_num - 1) * comm_radius

    adjacency = np.zeros((agent_num, agent_num))
    
    n = 0  # choose a communication point
    i_set = [k for k in range(comm_pos_set[n][0]-region_radius, comm_pos_set[n][0]+region_radius+1)]
    j_set = [k for k in range(comm_pos_set[n][1]-region_radius, comm_pos_set[n][1]+region_radius+1)]
    # generate search set
    search_point_set = []
    for i in i_set:
        for j in j_set:
            if (i,j) not in obstacle_points:
                search_point_set.append((i,j))

    max_distance = np.inf
    comm_pos_multi_set = [[[(0,0) for n in range(agent_num)]] for i in range(5)] #initial five position
    comm_agent_distance = [0.0 for i in range(5)]
    time_start = time.process_time()
  
  
    comm_pos_multi_set = [[(43, 54), (48, 54), (48, 49)], [(43, 54), (48, 54), (51, 50)], [(44, 54), (49, 54), (49, 49)], [(45, 54), (50, 54), (50, 49)], [(46, 52), (43, 55), (46, 47)]]
    m = 4
    comm_pos_multi = [(0,0) for i in range(agent_num*2)]
    for i in range(agent_num):
        comm_pos_multi[2*i] = comm_pos_multi_set[m][i]
        comm_pos_multi[2*i+1] = comm_pos_multi_set[m][i]
        
    finalpath = dijkstra.path_generate_multi(start_points, comm_pos_multi)
    
    
    # minmax = comm_agent_distance[4]
    # for i in range(4):
    #     if comm_agent_distance[i] > minmax:
    #         del comm_agent_distance[i]
    #         del comm_pos_multi_set[i]
    # print(comm_pos_multi_set, comm_agent_distance)


    #===========================================
    # Draw the result
    #===========================================
    fig = plt.figure(figsize = (10,10))
    ax = fig.add_subplot(111)
    plt.xlim(-1,101)
    plt.ylim(-1,101)
    plt.xticks(np.arange(0,105,5))
    plt.yticks(np.arange(0,105,5))

    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    plt.gcf().set_facecolor(np.ones(3)* 240 / 255)
    plt.grid(True, zorder=0)
    """
    # graph of the expansion obstacles
    for i in range(map_size):
        for j in range(map_size):
            if map.obstacle_map[i][j]:
                plt.plot(i,j,'sy', markersize=6)
    """
    # graph of the pure obstacles
    for i in range(len(obstacle_points)):
        plt.plot(obstacle_points[i][0],obstacle_points[i][1],'sk', markersize=6)

    for i in range(len(search_point_set)):
        plt.plot(search_point_set[i][0],search_point_set[i][1],'sy', markersize=3)   


    plt.plot(comm_pos_set[n][0], comm_pos_set[n][1], '.', markersize=10)
    circle_total = plt.Circle(comm_pos_set[n], region_radius, color='r', fill=False)
    plt.gcf().gca().add_artist(circle_total)

    # the generated final path
    colortype = ['blue', 'red', 'green']

    for j in range(0, agent_num*2, 2):
        plt.plot(comm_pos_multi[j][0], comm_pos_multi[j][1], '*', markersize=10)

        path_x, path_y = [],[]
        for k in range(len(finalpath[j])):
            path_x.append(finalpath[j][k][0])
            path_y.append(finalpath[j][k][1])
        plt.plot(path_x, path_y, linestyle='-', color=colortype[int(j/2)])
        
        path_x, path_y = [],[]
        for k in range(len(finalpath[j+1])):
            path_x.append(finalpath[j+1][k][0])
            path_y.append(finalpath[j+1][k][1])
        plt.plot(path_x, path_y, linestyle='-', color=colortype[int(j/2)])
        
        circle_agent = plt.Circle(comm_pos_multi[j], comm_radius/2, color='k', fill=False)
        plt.gcf().gca().add_artist(circle_agent)

    plt.title("Grid map simulation")
    plt.show()