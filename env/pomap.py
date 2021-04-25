import numpy as np
import os
import matplotlib.pyplot as plt


class POMap:
    def __init__(self, window: int, map_name: str):
        win = np.array([[i, j] for i in range(-window, window + 1) for j in range(-window, window + 1)]).T
        self.win_i = win[0]
        self.win_j = win[1]

        map_file = open(os.path.join('maps', map_name))
        map_file.readline()
        self.height = int(map_file.readline().split()[1])
        self.width = int(map_file.readline().split()[1])
        map_file.readline()
        self.view_cells = np.zeros((self.height, self.width))

        i = 0
        for line in map_file:
            j = 0
            for c in line:
                if c == '.':
                    self.view_cells[i][j] = 0
                elif c == 'T':
                    self.view_cells[i][j] = 1
                elif c == '@':
                    self.view_cells[i][j] = 2
                else:
                    continue
                j += 1

            if j != self.width:
                raise Exception("Size Error. Map width = ", j, ", but must be", self.width, "(map line: ", i, ")")

            i += 1
            if i == self.height:
                break
        map_file.close()

        self.cells = np.zeros((self.height, self.width))
        self.base_cells = np.zeros((self.height, self.width))
        self.observed = np.zeros((self.height, self.width))
        self.base_cells = (self.view_cells != 0).astype(int)

    def reset(self):
        self.cells = np.zeros((self.height, self.width))
        self.observed = np.zeros((self.height, self.width))

    def update(self, i: int, j: int):
        win_i = np.clip(self.win_i + i, 0, self.height - 1)
        win_j = np.clip(self.win_j + j, 0, self.width - 1)
        self.observed[win_i, win_j] = 1
        if np.all(self.cells[win_i, win_j] == self.base_cells[win_i, win_j]):
            return False
        else:
            self.cells[win_i, win_j] = self.base_cells[win_i, win_j]
            return True

    def in_bounds(self, i, j):
        return (0 <= j < self.width) and (0 <= i < self.height)

    def traversable(self, i, j):
        return not self.cells[i][j]

    def get_neighbors(self, i, j):
        neighbors = []
        delta = [[0, 1], [1, 0], [0, -1], [-1, 0]]
        for d in delta:
            if self.in_bounds(i + d[0], j + d[1]) and self.traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))

        delta_diag = [[1, 1], [1, -1], [-1, 1], [-1, -1]]
        for d in delta_diag:
            to_check = [[i + d[0], j], [i, j + d[1]]]
            flag = True
            for n in to_check:
                if not (self.in_bounds(n[0], n[1]) and self.traversable(n[0], n[1])):
                    flag = False
            if flag and self.in_bounds(i + d[0], j + d[1]) and self.traversable(i + d[0], j + d[1]):
                neighbors.append((i + d[0], j + d[1]))

        return neighbors

    def draw(self, start, goal, path: list):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[self.view_cells == 0, :] = 200
        image[self.view_cells == 1, 1] = 102
        image[self.observed == 1, :] += 50

        for step in path:
            if self.traversable(step[0], step[1]):
                image[step[0], step[1]] = [52, 152, 219]
            else:
                image[step[0], step[1]] = [230, 126, 34]

        image[start[0], start[1]] = [255, 255, 0]
        image[goal[0], goal[1]] = [231, 76, 60]

        fig, ax = plt.subplots(dpi=150, figsize=(self.width/10, self.height/10))
        ax.axes.xaxis.set_visible(False)
        ax.axes.yaxis.set_visible(False)
        image = plt.imshow(image).make_image(None)[0]
        plt.close()
        return image
