from typing import Tuple, Callable
from heapq import heappop, heappush
import math
from env.environment import Map, CalculateCost


class Node:
    def __init__(self, i=-1, j=-1, k=None, g=math.inf, h=math.inf, f=None, parent=None):
        self.i = i
        self.j = j
        self.g = g
        self.k = k
        self.h = h
        if f is None:
            self.f = self.g + h
        else:
            self.f = f
        self.parent = parent

    def __eq__(self, other):
        return (self.i == other.i) and (self.j == other.j)

    def __lt__(self, other):
        return self.f < other.f or ((self.f == other.f) and (self.h < other.h)) \
               or ((self.f == other.f) and (self.h == other.h) and (self.k > other.k))


class OpenList:
    def __init__(self):
        self.prioritized_heap = []
        self.ij_to_node = {}

    def __iter__(self):
        return iter(self.ij_to_node.values())

    def __len__(self):
        return len(self.ij_to_node)

    @property
    def is_empty(self):
        return len(self.ij_to_node) == 0

    def add_node(self, item):
        ij = (item.i, item.j)
        old_node = self.ij_to_node.get(ij)
        if old_node is None or item.g < old_node.g:
            self.ij_to_node[ij] = item
            heappush(self.prioritized_heap, item)

    def pop(self):
        best_node = heappop(self.prioritized_heap)
        ij = (best_node.i, best_node.j)
        while self.ij_to_node.pop(ij, None) is None:
            best_node = heappop(self.prioritized_heap)
            ij = (best_node.i, best_node.j)
        return best_node


class ClosedList:
    def __init__(self):
        self.elements = {}

    def __iter__(self):
        return iter(self.elements.values())

    def __len__(self):
        return len(self.elements)

    def add_node(self, item: Node):
        self.elements[(item.i, item.j)] = item

    def was_expanded(self, item: Node):
        return (item.i, item.j) in self.elements.keys()


class Astar:
    def __init__(self, gridmap: Map, start_coordinates: Tuple[int, int], goal_coordinates: Tuple[int, int],
                 heuristic: Callable[..., float]):
        self.open = OpenList()
        self.closed = ClosedList()
        self.gridmap = gridmap
        self.heuristic = heuristic

        start = Node(start_coordinates[0], start_coordinates[1],
                     g=0, h=heuristic(start_coordinates[0], start_coordinates[1],
                                      goal_coordinates[0], goal_coordinates[1]))
        self.goal = Node(goal_coordinates[0], goal_coordinates[1], h=0)
        self.open.add_node(start)

    def compute(self):
        k = 1
        while not self.open.is_empty:
            current = self.open.pop()
            self.closed.add_node(current)
            if current.i == self.goal.i  and current.j == self.goal.j:
                return True, current, self.closed, self.open
            for neighbor in self.gridmap.GetNeighbors(current.i, current.j):
                new_node = Node(neighbor[0], neighbor[1],
                                k=k,
                                g=current.g + CalculateCost(current.i, current.j,
                                                            neighbor[0], neighbor[1]),
                                h=self.heuristic(neighbor[0], neighbor[1],
                                                 self.goal.i, self.goal.j),
                                parent=current)

                if not self.closed.was_expanded(new_node):
                    self.open.add_node(new_node)
            k += 1
        return False, None, self.closed, self.open

    def make_path(self, goal):
        length = goal.g
        current = goal
        path = []
        while current.parent:
            path.append(current)
            current = current.parent
        path.append(current)
        return path[::-1], length
