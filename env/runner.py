from env.pomap import POMap
import os
from algorithms.Astar import Astar
from env.environment import ManhattanDistance, CalculateCost
import imageio


class TestMapRunner:
    def __init__(self, tasks: str, window: int):
        tasks_file = open(os.path.join('maps', tasks))
        map_name = tasks_file.readline()[:-1]
        self.tasks = [task.split() for task in tasks_file]
        tasks_file.close()
        self.grid_map = POMap(window, map_name)
        self.window = window

    def run(self, algorithm, start, goal, video=False):
        self.grid_map.reset()
        check = True
        length = 0
        stats = 0
        alg = algorithm(goal, ManhattanDistance)
        if video:
            filename = f'{alg}_{self.window}_{start}_{goal}.gif'
            frames = []

        while not (start == goal):
            if self.grid_map.update(start[0], start[1]) or check:
                check = False
                result = alg.compute(self.grid_map, start)
                stats += alg.statistics()
                if result[0]:
                    path, _ = alg.make_path(result[1])
                    if video:
                        frames.append(self.grid_map.draw(start, goal, path))
                    length += CalculateCost(start[0], start[1], path[1][0], path[1][1])
                    start = path[1]
                    path = path[1:]
                else:
                    print("Path not found!")
                    break
            else:
                if video:
                    frames.append(self.grid_map.draw(start, goal, path))
                length += CalculateCost(start[0], start[1], path[1][0], path[1][1])
                start = path[1]
                path = path[1:]
        if video:
            imageio.mimsave(filename, frames, duration=0.1)
        return length, stats

    def compute_tasks(self, algorithm):
        for ind, task in enumerate(self.tasks):
            length, stats = self.run(algorithm, (int(task[5]), int(task[4])), (int(task[7]), int(task[6])))
            print(f'{ind} {task[0]} Operations: {stats} Length: {length: .2f} Optimal: {float(task[8]): .2f}')

