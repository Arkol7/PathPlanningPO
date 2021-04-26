import pygame
import math
from queue import PriorityQueue
from heapq import heappop, heappush
import heapq

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def get_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():  # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():  # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][
            self.col + 1].is_barrier() and not grid[self.row + 1][self.col].is_barrier() and not grid[self.row][
            self.col + 1].is_barrier():  # DOWN-RIGHT
            self.neighbors.append(grid[self.row + 1][self.col + 1])

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][
            self.col - 1].is_barrier() and not grid[self.row + 1][self.col].is_barrier() and not grid[self.row][
            self.col - 1].is_barrier():  # DOWN-LEFT
            self.neighbors.append(grid[self.row + 1][self.col - 1])

        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][
            self.col + 1].is_barrier() and not grid[self.row - 1][self.col].is_barrier() and not grid[self.row][
            self.col + 1].is_barrier():  # UP-RIGHT
            self.neighbors.append(grid[self.row - 1][self.col + 1])

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier() and not \
                grid[self.row - 1][self.col].is_barrier() and not grid[self.row][self.col - 1].is_barrier():  # UP-LEFT
            self.neighbors.append(grid[self.row - 1][self.col - 1])

        return self.neighbors

    def __lt__(self, other):
        return False


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

class LPAStar:

    def __init__(self, grid, iStart, jStart, iGoal, jGoal, heuristic):
        self.height = WIDTH
        self.width = WIDTH
        self.iStart = iStart
        self.jStart = jStart
        self.iGoal = iGoal
        self.jGoal = jGoal
        self.h = heuristic
        self.U = Uheap()
        self.gridMap = grid
        self.rhs = {(i, j): math.inf for i in range(self.height) for j in range(self.width)}
        self.g = {(i, j): math.inf for i in range(self.height) for j in range(self.width)}
        self.rhs[(self.iStart, self.jStart)] = 0
        self.U.insert((self.iStart, self.jStart), self.calculateKey((self.iStart, self.jStart)))

    def calculateKey(self, state):
        g_rhs = min(self.g[state], self.rhs[state])
        return g_rhs + self.h(state[0], state[1], self.iGoal, self.jGoal), g_rhs

    def updateVertex(self, u, gridMap):
        if u != (self.iStart, self.jStart):
            prevKeys = [math.inf]
            for spot in gridMap[u[0]][u[1]].get_neighbors(gridMap):
                i = spot.row
                j = spot.col
                prevKeys.append(self.g[(i, j)] + CalculateCost(u[0], u[1], i, j))
            self.rhs[u] = min(prevKeys)
        self.U.remove(u)
        if self.g[u] != self.rhs[u]:
            self.U.insert(u, self.calculateKey(u))

    def computeShortestPath(self, draw, gridMap):
        goal = (self.iGoal, self.jGoal)
        while self.U.topKey() < self.calculateKey(goal) or self.rhs[goal] != self.g[goal]:

            u = self.U.pop()

            # gridMap[u[0]][u[1]].make_open()
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for spot in gridMap[u[0]][u[1]].get_neighbors(gridMap):
                    i = spot.row
                    j = spot.col
                    if (i, j) not in [(self.iStart, self.jStart), (self.iGoal, self.jGoal)]:
                        gridMap[i][j].make_open()
                    self.updateVertex((i, j), gridMap)
            else:
                self.g[u] = math.inf
                self.updateVertex(u, gridMap)
                for spot in gridMap[u[0]][u[1]].get_neighbors(gridMap):
                    i = spot.row
                    j = spot.col
                    if (i, j) not in [(self.iStart, self.jStart), (self.iGoal, self.jGoal)]:
                        gridMap[i][j].make_open()
                    self.updateVertex((i, j), gridMap)

            draw()

        return True

    def shortestPath(self, draw, gridMap):
        path = []
        state = (self.iGoal, self.jGoal)
        path.append(state)
        while state != (self.iStart, self.jStart):
            minimum = math.inf
            for spot in gridMap[state[0]][state[1]].get_neighbors(gridMap):
                i = spot.row
                j = spot.col
                if minimum > self.g[(i, j)] and (i, j) not in path:
                    minimum = self.g[(i, j)]
                    state = (i, j)
            if state != (self.iStart, self.jStart) and state != (self.iGoal, self.jGoal):
                gridMap[state[0]][state[1]].make_path()
                draw()
            path.append(state)
        return path[::-1]


def EuclidDistance(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)


def CalculateCost(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)
    lpa = None
    start = None
    end = None
    run = True
    count = 0
    while run:
        draw(win, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # LEFT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()
                    if lpa:
                        i, j = spot.row, spot.col
                        neig = [(i + 1,j + 1), (i + 1,j), (i,j + 1), (i - 1,j - 1), (i - 1,j), (i,j - 1), (i + 1,j - 1), (i - 1,j + 1)]
                        for n in neig:
                            if 0 < n[0] < spot.total_rows and 0 < n[1] < spot.total_rows:
                                lpa.updateVertex(n, grid)



            elif pygame.mouse.get_pressed()[2]:  # RIGHT
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    lpa = LPAStar(grid, start.row, start.col, end.row, end.col, EuclidDistance)
                    lpa.computeShortestPath(lambda: draw(win, grid, ROWS, width), grid)
                    shortest_path = lpa.shortestPath(lambda: draw(win, grid, ROWS, width), grid)
                if event.key == pygame.K_c:
                    for row in grid:
                        for spot in row:
                            if spot.is_open() or spot.color == PURPLE:
                                spot.reset()
                if event.key == pygame.K_r and lpa and shortest_path:
                    count += 1
                    lpa.computeShortestPath(lambda: draw(win, grid, ROWS, width), grid)
                    shortest_path = lpa.shortestPath(lambda: draw(win, grid, ROWS, width), grid)

    pygame.quit()


main(WIN, WIDTH)
