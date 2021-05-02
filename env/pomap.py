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
        self.changed_cells = None
        self.path = np.zeros((self.height, self.width))

    def reset(self):
        self.cells = np.zeros((self.height, self.width))
        self.observed = np.zeros((self.height, self.width))
        self.changed_cells = None
        self.path = np.zeros((self.height, self.width))

    def update(self, i: int, j: int):
        win_i = np.clip(self.win_i + i, 0, self.height - 1)
        win_j = np.clip(self.win_j + j, 0, self.width - 1)
        self.observed[win_i, win_j] = 1
        if np.all(self.cells[win_i, win_j] == self.base_cells[win_i, win_j]):
            self.changed_cells = None
            return False
        else:
            check = self.cells[win_i, win_j] != self.base_cells[win_i, win_j]
            check = np.argwhere(check)
            wi = win_i[check]
            wj = win_j[check]
            self.changed_cells = np.hstack((wi, wj))
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

        return neighbors

    def draw(self, start, goal):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.path[start[0],start[1]] = 1

        image[self.view_cells == 0, :] = 200
        image[self.view_cells == 1, 1] = 102
        image[self.observed == 1, :] += 50
        image[self.path == 1, :] = [0, 0, 255]

        image[start[0], start[1]] = [255, 255, 0]
        image[goal[0], goal[1]] = [231, 76, 60]

        fig, ax = plt.subplots(figsize=(self.width/10, self.height/10))
        ax.axes.xaxis.set_visible(False)
        ax.axes.yaxis.set_visible(False)
        image = plt.imshow(image).make_image(None)[0]
        plt.close()
        return image

    def draw_initial(self):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[self.view_cells == 0, :] = 200
        image[self.view_cells == 1, 1] = 102
        image += 50
        return image
