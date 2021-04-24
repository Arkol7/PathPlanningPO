import math
from PIL import Image, ImageDraw
import numpy as np
import matplotlib.pyplot as plt


class Map:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.cells = []

    def ReadFromString(self, cellStr, width, height):
        self.width = width
        self.height = height
        self.cells = [[0 for _ in range(width)] for _ in range(height)]
        cellLines = cellStr.split("\n")
        i = 0
        j = 0
        for l in cellLines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c == '.':
                        self.cells[i][j] = 0
                    elif c == '#':
                        self.cells[i][j] = 1
                    else:
                        continue
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width)

                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height)

    def SetGridCells(self, width, height, gridCells):
        self.width = width
        self.height = height
        self.cells = gridCells

    def inBounds(self, i, j):
        return (0 <= j < self.width) and (0 <= i < self.height)

    def Traversable(self, i, j):
        return not self.cells[i][j]

    def GetNeighbors(self, i, j):

        neighbors = []
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for d in delta:
            if self.inBounds(i + d[0], j + d[1]) and self.Traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))

        delta_diag = [[1, 1], [1, -1], [-1, 1], [-1, -1]]
        for d in delta_diag:
            to_check = [[i + d[0], j], [i, j + d[1]]]
            flag = True
            for n in to_check:
                if not (self.inBounds(n[0], n[1]) and self.Traversable(n[0], n[1])):
                    flag = False
            if flag and self.inBounds(i + d[0], j + d[1]) and self.Traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))

        return neighbors


class Node:
    def __init__(self, i=-1, j=-1, g=math.inf, h=math.inf, rhs=math.inf, F=None, parent=None):
        self.i = i
        self.j = j
        self.g = g
        self.rhs = rhs
        self.h = h
        self.key = (min(self.g, self.rhs) + self.h, min(self.g, self.rhs))
        if F is None:
            self.F = self.g + h
        else:
            self.F = F
        self.parent = parent

    def __eq__(self, other):
        return (self.i == other.i) and (self.j == other.j)

    def __lt__(self, other):
        return self.key < other.key


def Draw(gridMap: Map, start: Node = None, goal: Node = None, path: list = None, nodesExpanded=None, nodesOpened=None):
    k = 5
    hIm = gridMap.height * k
    wIm = gridMap.width * k
    im = Image.new('RGB', (wIm, hIm), color='white')
    draw = ImageDraw.Draw(im)
    for i in range(gridMap.height):
        for j in range(gridMap.width):
            if (gridMap.cells[i][j] == 1):
                draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=(70, 80, 80))

    if nodesOpened is not None:
        for node in nodesOpened:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(213, 219, 219),
                           width=0)

    if nodesExpanded is not None:
        for node in nodesExpanded:
            draw.rectangle((node.j * k, node.i * k, (node.j + 1) * k - 1, (node.i + 1) * k - 1), fill=(131, 145, 146),
                           width=0)

    if path is not None:
        for step in path:
            if (step is not None):
                if (gridMap.Traversable(step.i, step.j)):
                    draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1),
                                   fill=(52, 152, 219), width=0)
                else:
                    draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1),
                                   fill=(230, 126, 34), width=0)

    if (start is not None) and (gridMap.Traversable(start.i, start.j)):
        draw.rectangle((start.j * k, start.i * k, (start.j + 1) * k - 1, (start.i + 1) * k - 1), fill=(40, 180, 99),
                       width=0)

    if (goal is not None) and (gridMap.Traversable(goal.i, goal.j)):
        draw.rectangle((goal.j * k, goal.i * k, (goal.j + 1) * k - 1, (goal.i + 1) * k - 1), fill=(231, 76, 60),
                       width=0)

    fig, ax = plt.subplots(dpi=150)
    ax.axes.xaxis.set_visible(False)
    ax.axes.yaxis.set_visible(False)
    plt.imshow(np.asarray(im))


def ManhattanDistance(i1, j1, i2, j2):
    return abs(int(i1) - int(i2)) + abs(int(j1) - int(j2))


def DiagonalDistance(i1, j1, i2, j2):
    return abs(abs(int(i1) - int(i2)) - abs(int(j1) - int(j2))) + min(abs(int(i1) - int(i2)), abs(int(j1) - int(j2)))


def ChebyshevDistance(i1, j1, i2, j2):
    return max([abs(int(i1) - int(i2)), abs(int(j1) - int(j2))])


def EuclidDistance(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)


def MakePath(goal):
    length = goal.rhs
    current = goal
    path = []
    while current.parent:
        path.append(current)
        current = current.parent
    path.append(current)
    return path[::-1], length


def CalculateCost(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)