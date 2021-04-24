import heapq
import math
from env.environment import CalculateCost
from env.environment import Node


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
        self.remove(item)
        self.insert(item, priority)

    def remove(self, state):
        for index, (_, _, item) in enumerate(self.heap):
            if item == state:
                del self.heap[index]


class LPAStar:

    def __init__(self, gridMap, iStart, jStart, iGoal, jGoal, heuristic):
        self.height = gridMap.height
        self.width = gridMap.width
        self.iStart = iStart
        self.jStart = jStart
        self.iGoal = iGoal
        self.jGoal = jGoal
        self.h = heuristic
        self.U = Uheap()
        self.gridMap = gridMap
        self.rhs = {(i, j): math.inf for i in range(self.height) for j in range(self.width)}
        self.g = {(i, j): math.inf for i in range(self.height) for j in range(self.width)}
        self.rhs[(self.iStart, self.jStart)] = 0
        self.U.insert((self.iStart, self.jStart), self.calculateKey((self.iStart, self.jStart)))

    def calculateKey(self, state):
        g_rhs = min(self.g[state], self.rhs[state])
        return (g_rhs + self.h(state[0], state[1], self.iGoal, self.jGoal), g_rhs)

    def updateVertex(self, u):
        if u != (self.iStart, self.jStart):
            prevKeys = [math.inf]
            for (i, j) in self.gridMap.GetNeighbors(u[0], u[1]):
                prevKeys.append(self.g[(i, j)] + CalculateCost(u[0], u[1], i, j))
            self.rhs[u] = min(prevKeys)
        self.U.remove(u)
        if self.g[u] != self.rhs[u]:
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self):
        goal = (self.iGoal, self.jGoal)
        while self.U.topKey() < self.calculateKey(goal) or self.rhs[goal] != self.g[goal]:
            u = self.U.pop()
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for (i, j) in self.gridMap.GetNeighbors(u[0], u[1]):
                    self.updateVertex((i, j))
            else:
                self.g[u] = math.inf
                self.updateVertex(u)
                for (i, j) in self.gridMap.GetNeighbors(u[0], u[1]):
                    self.updateVertex((i, j))
        return True, Node(u[0], u[1]), self.U

    def shortestPath(self):
        path = []
        state = (self.iGoal, self.jGoal)
        path.append(state)
        while state != (self.iStart, self.jStart):
            minimum = math.inf
            for (i, j) in self.gridMap.GetNeighbors(state[0], state[1]):
                if minimum > self.g[(i, j)]:
                    minimum = self.g[(i, j)]
                    state = (i, j)
            path.append(state)

        res = []
        for n in path[::-1]:
            res.append(Node(n[0], n[1]))
        return res
        return path[::-1]

