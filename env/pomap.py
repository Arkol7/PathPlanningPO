import numpy as np
from PIL import Image, ImageDraw


class POMap:
    def __init__(self, window: int):
        win = np.array([[i, j] for i in range(-window, window + 1) for j in range(-window, window + 1)]).T
        self.win_i = win[0]
        self.win_j = win[1]

    def ReadFromString(self, cell_string: str, width: int, height: int):
        self.width = width
        self.height = height
        self.cells = np.zeros((height, width))
        self.base_cells = np.zeros((height, width))
        self.observed = np.zeros((height, width))
        cell_lines = cell_string.split("\n")
        i = 0
        j = 0
        for l in cell_lines:
            if len(l) != 0:
                j = 0
                for c in l:
                    if c == '.':
                        self.base_cells[i][j] = 0
                    elif c == '#':
                        self.base_cells[i][j] = 1
                    else:
                        continue
                    j += 1
                if j != width:
                    raise Exception("Size Error. Map width = ", j, ", but must be", width)

                i += 1

        if i != height:
            raise Exception("Size Error. Map height = ", i, ", but must be", height)

    def update(self, i: int, j: int):
        win_i = np.clip(self.win_i + i, 0, self.height - 1)
        win_j = np.clip(self.win_j + j, 0, self.width - 1)
        self.observed[win_i, win_j] = 1
        if np.all(self.cells[win_i, win_j] == self.base_cells[win_i, win_j]):
            return False
        else:
            self.cells[win_i, win_j] = self.base_cells[win_i, win_j]
            return True

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

    def draw(self, start = None, goal = None, path: list = None):
        k = 10
        hIm = self.height * k
        wIm = self.width * k
        im = Image.new('RGB', (wIm, hIm), color='white')
        draw = ImageDraw.Draw(im)
        for i in range(self.height):
            for j in range(self.width):
                if (self.cells[i][j] == 1) and (self.base_cells[i][j] == 1):
                    draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=(40, 50, 50))
                elif self.base_cells[i][j] == 1:
                    draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=(100, 110, 110))
                elif self.observed[i][j] == 1:
                    draw.rectangle((j * k, i * k, (j + 1) * k - 1, (i + 1) * k - 1), fill=(200, 200, 255))

        if path is not None:
            for step in path:
                if (step is not None):
                    if (self.Traversable(step.i, step.j)):
                        draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1),
                                       fill=(52, 152, 219), width=0)
                    else:
                        draw.rectangle((step.j * k, step.i * k, (step.j + 1) * k - 1, (step.i + 1) * k - 1),
                                       fill=(230, 126, 34), width=0)

        if (start is not None) and (self.Traversable(start.i, start.j)):
            draw.rectangle((start.j * k, start.i * k, (start.j + 1) * k - 1, (start.i + 1) * k - 1), fill=(40, 180, 99),
                           width=0)

        if (goal is not None) and (self.Traversable(goal.i, goal.j)):
            draw.rectangle((goal.j * k, goal.i * k, (goal.j + 1) * k - 1, (goal.i + 1) * k - 1), fill=(231, 76, 60),
                           width=0)

        return np.asarray(im)
