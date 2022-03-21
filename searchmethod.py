# -*- coding: utf-8 -*-

from importlib.resources import path
import math
from sys import prefix
import numpy as np


class Dijkstra:
    
    def __init__(self, obstacle_map):

        self.obstacle_map = obstacle_map
        self.x_width, self.y_width = np.shape(self.obstacle_map)
        self.motion = self.motion_model()

    class Node:
        """
        Define each grid as a node
        """
        def __init__(self, current_pos, cost, pre_index):
            """
            Args:
                current_node (2-3 tuple): current node with (x,y), [m]
                cost (float): distance to start node
                pre_index (int): the index of the previous node
            """
            self.x = current_pos[0]
            self.y = current_pos[1]
            self.cost = cost
            self.pre_index = pre_index
        
        def __str__(self):
            return str((self.x, self.y)) + "," + str(self.cost) \
                    + "," + str(self.pre_index)


    def planning_goal(self, start_pos, goal_pos):
        """
        Dijkstra path serach, find the shortest path from start position to goal position for single agent.

        Input:
            start_pos (2-d tuple): start position (x,y), [m]
            goal_pos (2-d tuple): goal position (x,y), [m]

        Output:
            finalpath (a list of 2-d tuples): the final path from goal to start
        """
        start_node = self.Node(start_pos, 0.0, -1)
        goal_node = self.Node(goal_pos, 0.0, -1)

        open_set, closed_set = dict(), dict()  # key-value: hash list
        open_set[self.index(start_node)] = start_node

        while open_set:
                
            mincost_index = min(open_set, key=lambda i: open_set[i].cost)  # find the node with the minimal cost
            current_node = open_set[mincost_index]

            # determine whether it is the goal node
            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Find the goal " + str((goal_node.x, goal_node.y)))
                #goal_node = current_node
                goal_node.pre_index = current_node.pre_index
                goal_node.cost = current_node.cost
                return self.path_generate(goal_node, closed_set)
            
            # remove the item from the open set
            del open_set[mincost_index]

            # add it to the closed set
            closed_set[mincost_index] = current_node

            # expend search grid based o motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node((current_node.x + move_x, current_node.y + move_y),
                                 current_node.cost + move_cost,
                                 mincost_index)
                node_index = self.index(node)

                if node_index in closed_set:
                    continue
                
                if not self.if_obstacle(node):
                    continue

                if node_index not in open_set:
                    open_set[node_index] = node  # discover a new node
                else:
                    if open_set[node_index].cost >= node.cost:
                        open_set[node_index] = node
        print('There is no path!!!')
        return None


    def planning_map(self, start_pos):
        """
        Dijkstra path serach, set shortest distance on each grid of the map for sigle agent.

        Input:
            start_pos (2-d tuple): start position (x,y), [m]

        Output:
            closed_set (a dictionary of form {index: pos, cost, index})
        """

        start_node = self.Node(start_pos, 0.0, -1)

        open_set, closed_set = dict(), dict()  # key-value: hash list
        open_set[self.index(start_node)] = start_node

        while True:
            # determine whether it has traversed the whole map
            if not open_set:
                print("All grids haven been traversed!")
                break

            mincost_index = min(open_set, key=lambda i: open_set[i].cost)  # find the node with the minimal cost
            current_node = open_set[mincost_index]
            
            # remove the item from the open set
            del open_set[mincost_index]

            # add it to the closed set
            closed_set[mincost_index] = current_node
            # expend search grid based o motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node((current_node.x + move_x, current_node.y + move_y),
                                 current_node.cost + move_cost,
                                 mincost_index)
                node_index = self.index(node)

                if node_index in closed_set:
                    continue
                
                if not self.if_obstacle(node):
                    continue

                if node_index not in open_set:
                    open_set[node_index] = node  # discover a new node
                else:
                    if open_set[node_index].cost >= node.cost:
                        open_set[node_index] = node

        return closed_set


    def planning_map_multiagent(self, start_pos_set):
        """
        Dijkstra path serach, set shortest distance on each grid of the map for multi agents.

        Input:
            start_pos_set (a list of 2-d tuple): start position of multi agents [(x1,y1),(x2,y2),...], [m]

        Output:
            closed_set (a dictionary of form {index: pos, cost, index})
        """
        agent_num = len(start_pos_set)
        costmap_dict_multi = []
        for i in range(agent_num):
            costmap_dict_multi.append(self.planning_map(start_pos_set[i]))
        
        # transer 'costmap_dict' to 'costmap_list'
        # initial a 'agent_num'-d list [[[None]], [[None]], [[None]], ...]
        #costmap_list_multi = [[[np.Inf] * self.y_width] * self.x_width] * agent_num # stupid method, it copies all the change

        costmap_list_multi = [[[np.inf for t in range(self.y_width)] for i in range(self.x_width)] for i in range(agent_num)]
        costmap_xy_multi = [[] for i in range(agent_num)]

        for i in range(agent_num):
            for node in costmap_dict_multi[i].values():
                costmap_list_multi[i][node.x][node.y] = node.cost
                costmap_xy_multi[i].append((node.x, node.y))
                axis = costmap_list_multi[i]
                bxis = costmap_xy_multi[i]

        return costmap_dict_multi, costmap_list_multi, costmap_xy_multi


    def min_max_distance(self, start_pos_set, costmap_list_multi=None):
        """
        Find the communication position, where the maximal distance of all agents from start to communication is minimized.

        Input:
            start_pos_set (a list of 2-d tuple): start position of multi agents [(x1,y1),(x2,y2),...], [m]
            costmap_list_multi: the costmap of the multi start position
        Output:
            comm_pos_set (a list of 2-d tuple): the optimized communication position
            agent_distance (a list of float): the distance of ech agent
        """
        agent_num = int(len(start_pos_set)/2)
        if not costmap_list_multi:
            costmap_dict_multi, costmap_list_multi, costmap_xy_multi = self.planning_map_multiagent(start_pos_set)

        max_distance = np.inf
        for i in range(self.x_width):
            for j in range(self.y_width):
                if not self.obstacle_map[i][j]:
                    p_distance = [0 for i in range(agent_num)]
                    for n in range(0, agent_num*2, 2):
                        p_distance[int(n/2)] = costmap_list_multi[n][i][j] + costmap_list_multi[n+1][i][j]
                    
                    if max(p_distance) < max_distance:
                        max_distance = max(p_distance)
                        agent_max = p_distance.index(max_distance) # find the agent with the max distance

        comm_pos_set = []
        agent_distance = []
        for i in range(self.x_width):
            for j in range(self.y_width):
                if not self.obstacle_map[i][j]:
                    p_distance = [0 for i in range(agent_num)]
                    for n in range(0, agent_num*2, 2):
                        p_distance[int(n/2)] = costmap_list_multi[n][i][j] + costmap_list_multi[n+1][i][j]
                    
                    if max(p_distance) == max_distance:
                        comm_pos_set.append((i, j))
                        agent_distance.append(p_distance)

        print("The max distance is %.2f m." %max_distance)
        print("The position can be " + str(comm_pos_set))
        return comm_pos_set, agent_distance


    def index(self, node):

        return node.y * self.x_width + node.x


    def if_obstacle(self, node):
        
        if node.x <= 0 or node.y <= 0:
            return False

        if node.x >= self.x_width or node.y >= self.y_width:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False
        
        return True


    def path_generate(self, goal_node, closed_set):
        
        index = goal_node.pre_index
        finalpath = [(goal_node.x, goal_node.y)]

        while index != -1:
            node = closed_set[index]
            finalpath.append((node.x, node.y))
            index = node.pre_index

        return finalpath

    def path_generate_multi(self, start_pos_set, goal_pos_set):
        """
        Generate multi final path to all goals for all agents.

        Input:
            start_pos_set (a list of 2-d tuple): start position of multi agents [(x1,y1),(x2,y2),...], [m]
            goal_pos_set (a list of 2-d tuple): multi goal position [(x1,y1),(x2,y2),...], [m]
            Notice: the number of 'start_pos_set' and 'goal_pos_set' should be the same!

        Output:
            finalpath (a list of 2-d tuple with goal_num*agent_num dimensional)
        """       
        agent_num = len(start_pos_set)

        finalpath = [[] for i in range(agent_num)]

        for i in range(agent_num):
            finalpath[i] = self.planning_goal(start_pos_set[i], goal_pos_set[i])
        
        return finalpath


    def motion_model(self):

        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
