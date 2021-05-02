from typing import Tuple, Callable
from heapq import heappop, heappush
import heapq
import math
from env.environment import CalculateCost
from env.pomap import POMap
import numpy as np


class Uheap:

    def __init__(self):
        self.heap = []
        self.count = 0

    def __len__(self):
        return len(self.heap)

    def top(self):
        return self.heap[0]

    def topKey(self):
        if self.isEmpty():
            return (math.inf, math.inf)
        return self.heap[0][0]

    def insert(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.insert(item, priority)

    def remove(self, state):
        self.update(state, (-math.inf, -math.inf))
        a = self.pop()


class DstarLite:
    def __init__(self, gridmap: POMap, start_coordinates: Tuple[int, int], goal_coordinates: Tuple[int, int],
                 heuristic: Callable[..., float]):
        self.h = heuristic
        self.goal = goal_coordinates
        self.start = start_coordinates
        self.gridmap = gridmap
        self.nodes = 0

        self.k_m = 0
        self.U = Uheap()
        self.rhs = {(i, j): math.inf for i in range(self.gridmap.height) for j in range(self.gridmap.width)}
        self.g = {(i, j): math.inf for i in range(self.gridmap.height) for j in range(self.gridmap.width)}
        self.rhs[self.goal] = 0
        self.nodes += 1
        self.U.insert(self.goal, self.calculateKey(self.goal))

    def calculateKey(self, state: Tuple[int, int]):
        g_rhs = min(self.g[state], self.rhs[state])
        return g_rhs + self.h(state[0], state[1], self.start[0], self.start[1])+self.k_m, g_rhs

    def updateVertex(self, u):
        if u != self.goal:
            prevKeys = [math.inf]
            for neighbor in self.gridmap.get_neighbors(u[0], u[1]):
                prevKeys.append(self.g[neighbor] + CalculateCost(u[0], u[1],
                                                                 neighbor[0],
                                                                 neighbor[1]))
            self.rhs[u] = min(prevKeys)
        self.U.remove(u)
        if self.g[u] != self.rhs[u]:
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self):
        while self.U.topKey() < self.calculateKey(self.start) or self.rhs[self.start] != self.g[self.start]:
            k_old = self.U.topKey()
            u = self.U.pop()

            if k_old < self.calculateKey(u):
                self.nodes += 1
                self.U.insert(u, self.calculateKey(u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for neighbor in self.gridmap.get_neighbors(u[0], u[1]):
                    self.nodes += 1
                    self.updateVertex(neighbor)
            else:
                self.g[u] = math.inf
                self.nodes += 1
                self.updateVertex(u)
                for neighbor in self.gridmap.get_neighbors(u[0], u[1]):
                    self.nodes += 1
                    self.updateVertex(neighbor)
        return True, self.shortestPath()

    def shortestPath(self):
        path = []
        state = self.start
        path.append(state)
        while state != self.goal:
            minimum = math.inf
            for neighbor in self.gridmap.get_neighbors(state[0], state[1]):
                if minimum > self.g[neighbor] and neighbor not in path:
                    minimum = self.g[neighbor]
                    state = neighbor
            path.append(state)
        return path

    def __str__(self):
        return 'DstarLite'

    def reset(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        pass

    def compute(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        if start_coordinates == self.start:
            check, path = self.computeShortestPath()
        else:
            self.k_m += self.h(self.start[0], self.start[1],
                               start_coordinates[0], start_coordinates[1])
            self.start = start_coordinates
            self.gridmap = gridmap
            if not(self.gridmap.changed_cells is None):
                for vertex in self.gridmap.changed_cells:
                    i, j = vertex[0], vertex[1]
                    neig = [(i + 1, j), (i, j + 1), (i - 1, j), (i, j - 1)]

                    for neighbor in neig:
                        if 0 <= neighbor[0] < self.gridmap.height and 0 <= neighbor[1] < self.gridmap.width:
                            self.nodes += 1
                            self.updateVertex(neighbor)
            check, path = self.computeShortestPath()
        return check, path, self.nodes
