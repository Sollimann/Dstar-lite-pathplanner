from priority_queue import PriorityQueue, Priority
from grid import OccupancyGridMap
import numpy as np
from utils import heuristic, Vertex, Vertices
from typing import Dict, List

OBSTACLE = 255
UNOCCUPIED = 0


class DStarLite:
    def __init__(self, map: OccupancyGridMap, s_start: (int, int), s_goal: (int, int), view_range=2):
        """
        :param map: the ground truth map of the environment provided by gui
        :param s_start: start location
        :param s_goal: end location
        :param view_range: how far ahead we scan for changed cells
        """
        self.view_range = view_range
        self.s_start = s_start
        self.s_goal = s_goal
        self.s_last = s_start
        self.k_m = 0  # accumulation
        self.U = PriorityQueue()
        self.rhs = np.ones((map.x_dim, map.y_dim)) * np.inf
        self.g = self.rhs.copy()

        self.global_map = map
        self.sensed_map = OccupancyGridMap(x_dim=map.x_dim,
                                           y_dim=map.y_dim,
                                           exploration_setting='8N')

        self.rhs[self.s_goal] = 0
        self.U.insert(self.s_goal, Priority(heuristic(self.s_start, self.s_goal), 0))

    def calculate_key(self, s: (int, int)):
        """
        :param s: the vertex we want to calculate key
        :return: Priority class of the two keys
        """
        k1 = min(self.g[s], self.rhs[s]) + heuristic(self.s_start, s) + self.k_m
        k2 = min(self.g[s], self.rhs[s])
        return Priority(k1, k2)

    def c(self, u: (int, int), v: (int, int)) -> float:
        """
        calcuclate the cost between nodes
        :param u: from vertex
        :param v: to vertex
        :return: euclidean distance to traverse. inf if obstacle in path
        """
        if not self.sensed_map.is_unoccupied(u) or not self.sensed_map.is_unoccupied(v):
            return float('inf')
        else:
            return heuristic(u, v)

    def contain(self, u: (int, int)) -> (int, int):
        return u in self.U.vertices_in_heap

    def update_vertex(self, u: (int, int)):
        if self.g[u] != self.rhs[u] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            self.U.remove(u)

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                pred = self.sensed_map.succ(vertex=u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s] = min(self.rhs[s], self.c(s, u) + self.g[u])
                    self.update_vertex(s)
            else:
                self.g_old = self.g[u]
                self.g[u] = float('inf')
                pred = self.sensed_map.succ(vertex=u)
                pred.append(u)
                for s in pred:
                    if self.rhs[s] == self.c(s, u) + self.g_old:
                        if s != self.s_goal:
                            min_s = float('inf')
                            succ = self.sensed_map.succ(vertex=s)
                            for s_ in succ:
                                temp = self.c(s, s_) + self.g[s_]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[s] = min_s
                    self.update_vertex(u)

    def update_changed_edge_costs(self, local_grid: Dict) -> Vertices:
        vertices = Vertices()
        for node, value in local_grid.items():
            if value == OBSTACLE:
                if self.sensed_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.sensed_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)

                    # add the obstacle
                    self.sensed_map.set_obstacle(node)
            else:
                if not self.sensed_map.is_unoccupied(node):
                    v = Vertex(pos=node)
                    succ = self.sensed_map.succ(node)
                    for u in succ:
                        v.add_edge_with_cost(succ=u, cost=self.c(u, v.pos))
                    vertices.add_vertex(v)

                    # remove the obstacle
                    self.sensed_map.remove_obstacle(node)
        return vertices

    def rescan(self, global_position: (int, int), update_globally: bool):

        if update_globally:
            # rescan local area
            local_observation = self.global_map.local_observation(global_position=global_position,
                                                                  view_range=self.view_range)
        else:
            # rescan local area
            local_observation = self.sensed_map.local_observation(global_position=global_position,
                                                                  view_range=self.view_range)

        return self.update_changed_edge_costs(local_grid=local_observation)

    def move_and_replan(self, robot_position: (int, int)):
        path = [robot_position]
        self.s_start = robot_position
        self.s_last = self.s_start
        self.rescan(global_position=robot_position, update_globally=True)
        self.compute_shortest_path()

        while self.s_start != self.s_goal:
            assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"

            succ = self.sensed_map.succ(self.s_start, avoid_obstacles=False)
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.c(self.s_start, s_) + self.g[s_]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            ### algorithm sometimes gets stuck here for some reason !!! FIX
            self.s_start = arg_min
            path.append(self.s_start)
            # scan graph for changed costs
            changed_edges_with_old_cost = self.rescan(global_position=self.s_start, update_globally=True)

            # if any edge costs changed
            if changed_edges_with_old_cost:
                self.k_m += heuristic(self.s_last, self.s_start)
                self.s_last = self.s_start

                # for all directed edges (u,v) with changed edge costs
                vertices = changed_edges_with_old_cost.vertices
                for vertex in vertices:
                    v = vertex.pos
                    succ_v = vertex.edges_and_c_old
                    for u, c_old in succ_v.items():
                        c_new = self.c(u, v)
                        if c_old > c_new:
                            if u != self.s_goal:
                                self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
                        elif self.rhs[u] == c_old + self.g[v]:
                            if u != self.s_goal:
                                min_s = float('inf')
                                succ_u = self.sensed_map.succ(vertex=u)
                                for s_ in succ_u:
                                    temp = self.c(u, s_) + self.g[s_]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.update_vertex(u)
                self.compute_shortest_path()
        print("path found!")
        return path, self.g, self.rhs
