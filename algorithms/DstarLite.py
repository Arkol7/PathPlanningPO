from typing import Tuple, Callable
import heapq
import math
from env.pomap import POMap


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
        self.last = start_coordinates
        self.gridmap = gridmap
        self.accesses = 0
        self.expansions = 0
        self.percolates = 0

        self.k_m = 0
        self.U = Uheap()
        self.rhs = dict()
        self.g = dict()
        self.rhs[self.goal] = 0
        self.accesses += 1
        self.percolates += 1
        self.U.insert(self.goal, self.calculateKey(self.goal))

    def calculateKey(self, state: Tuple[int, int]):
        g_rhs = min(self.g.get(state, math.inf), self.rhs.get(state, math.inf))
        return g_rhs + self.h(state[0], state[1], self.start[0], self.start[1]) + self.k_m, g_rhs

    def updateVertex(self, u):
        if u != self.goal:
            prevKeys = math.inf
            for neighbor in self.gridmap.get_all_neighbors(u[0], u[1]):
                self.accesses += 1
                f = self.g.get(neighbor, math.inf) + self.gridmap.calculate_cost(u, neighbor)
                if f < prevKeys:
                    prevKeys = f
            self.rhs[u] = prevKeys
        self.U.remove(u)
        if self.g.get(u, math.inf) != self.rhs.get(u, math.inf):
            self.percolates += 1
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self):
        while self.U.topKey() < self.calculateKey(self.start) or self.rhs.get(self.start,
                                                                              math.inf) != self.g.get(self.start,
                                                                                                      math.inf):
            k_old = self.U.topKey()
            u = self.U.pop()
            self.expansions += 1
            self.accesses += 1

            if k_old < self.calculateKey(u):
                self.percolates += 1
                self.U.insert(u, self.calculateKey(u))
            elif self.g.get(u, math.inf) > self.rhs.get(u, math.inf):
                self.g[u] = self.rhs.get(u, math.inf)
                for neighbor in self.gridmap.get_all_neighbors(u[0], u[1]):
                    self.accesses += 1
                    self.updateVertex(neighbor)
            else:
                self.g[u] = math.inf
                self.updateVertex(u)
                for neighbor in self.gridmap.get_all_neighbors(u[0], u[1]):
                    self.accesses += 1
                    self.updateVertex(neighbor)
        return True

    def shortestPath(self):
        path = []
        state = self.start
        path.append(state)
        while state != self.goal:
            minimum = math.inf
            for neighbor in self.gridmap.get_neighbors(state[0], state[1]):
                if minimum > self.g.get(neighbor, math.inf) and neighbor not in path:
                    minimum = self.g.get(neighbor, math.inf)
                    state = neighbor
            path.append(state)
        return path

    def __str__(self):
        return 'DstarLite'

    def reset(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        pass

    def compute(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        check = True
        if start_coordinates == self.start:
            check = self.computeShortestPath()
        else:
            self.start = start_coordinates
            self.gridmap = gridmap
            if not (self.gridmap.changed_cells is None):
                self.k_m += self.h(self.last[0], self.last[1],
                                   start_coordinates[0], start_coordinates[1])
                self.last = start_coordinates
                for vertex in self.gridmap.changed_cells:
                    self.accesses += 1
                    self.updateVertex((vertex[0], vertex[1]))
                    for vertex_neighbor in self.gridmap.get_all_neighbors(vertex[0], vertex[1]):
                        self.updateVertex(vertex_neighbor)
                        self.accesses += 1

                check = self.computeShortestPath()
        state = None
        minimum = math.inf
        for neighbor in self.gridmap.get_all_neighbors(start_coordinates[0],
                                                   start_coordinates[1]):
            if minimum > self.g.get(neighbor, math.inf) + self.gridmap.calculate_cost(start_coordinates,
                                                                                      neighbor):
                minimum = self.g.get(neighbor, math.inf) + self.gridmap.calculate_cost(start_coordinates,
                                                                                      neighbor)
                state = neighbor
        return check, state
