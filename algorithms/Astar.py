from typing import Tuple, Callable
from heapq import heappop, heappush
import math
from common.utils import CalculateCost
from env.pomap import POMap


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
        return self.f < other.f or ((self.f == other.f) and (self.g < other.g)) \
               or ((self.f == other.f) and (self.g == other.g) and (self.k > other.k))


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
    def __init__(self, gridmap: POMap, start_coordinates: Tuple[int, int],
                 goal_coordinates: Tuple[int, int], heuristic: Callable[..., float]):
        self.accesses = 0
        self.expansions = 0
        self.insertions = 0

        self.open = OpenList()
        self.closed = ClosedList()
        self.gridmap = gridmap
        self.heuristic = heuristic
        self.goal = start_coordinates
        self.start = Node(goal_coordinates[0], goal_coordinates[1], k=0)
        self.start.g = 0
        self.start.h = self.heuristic(self.start.i, self.start.j,
                                      self.goal[0], self.goal[1])
        self.start.f = self.start.h
        self.accesses += 1
        self.insertions += 1
        self.open.add_node(self.start)

    def __str__(self):
        return 'Astar'

    def reset(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        self.open = OpenList()
        self.closed = ClosedList()
        self.gridmap = gridmap
        self.goal = start_coordinates
        self.start.g = 0
        self.start.h = self.heuristic(self.start.i, self.start.j,
                                      self.goal[0], self.goal[1])
        self.start.f = self.start.h
        self.accesses += 1
        self.insertions += 1
        self.open.add_node(self.start)

    def compute(self, gridmap: POMap, start_coordinates: Tuple[int, int]):
        check0 = self.goal == start_coordinates
        check = True
        if not(self.gridmap.changed_cells is None) or check0:
            if not check0:
                self.reset(gridmap, start_coordinates)
            k = 1
            check = False
            while not self.open.is_empty:
                current = self.open.pop()
                self.accesses += 1
                self.expansions += 1
                self.closed.add_node(current)
                for neighbor in self.gridmap.get_neighbors(current.i, current.j):
                    new_node = Node(neighbor[0], neighbor[1],
                                    k=k,
                                    g=current.g + CalculateCost(current.i, current.j,
                                                                neighbor[0], neighbor[1]),
                                    h=self.heuristic(neighbor[0], neighbor[1],
                                                     self.goal[0], self.goal[1]),
                                    parent=current)
                    self.accesses += 1
                    if not self.closed.was_expanded(new_node):
                        self.insertions += 1
                        self.open.add_node(new_node)
                if current.i == self.goal[0] and current.j == self.goal[1]:
                    check = True
                    break
                k += 1

        if not check:
            return False, None

        minimum = math.inf
        state = None
        for neighbor in self.gridmap.get_neighbors(start_coordinates[0],
                                                   start_coordinates[1]):
            cur = self.closed.elements.get(neighbor)
            if not (cur is None):
                if minimum > cur.g + CalculateCost(cur.i, cur.j, start_coordinates[0], start_coordinates[1]):
                    minimum = cur.g + CalculateCost(cur.i, cur.j, start_coordinates[0], start_coordinates[1])
                    state = neighbor
        if state is None:
            raise Exception('Couldn\'t find next step')
        return check, state


