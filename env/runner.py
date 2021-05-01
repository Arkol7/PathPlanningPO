from typing import Tuple
from env.pomap import POMap
import os
from algorithms.Astar import Astar
from algorithms.DstarLite import DstarLite
from common.utils import ManhattanDistance, CalculateCost
import imageio
from common.utils import timed
import wandb


class TestMapRunner:
    def __init__(self, tasks: str, window: int):
        tasks_file = open(os.path.join('maps', tasks))
        self.map_name = tasks_file.readline()[:-1]
        self.tasks = [task.split() for task in tasks_file]
        tasks_file.close()
        self.grid_map = POMap(window, self.map_name)
        self.window = window

    @timed
    def run(self, algorithm, start: Tuple[int, int], goal: Tuple[int, int], video: bool = False):
        self.grid_map.reset()
        check = True
        stats = {'Used nodes': 0, 'length': 0}
        alg = algorithm(self.grid_map, start, goal, ManhattanDistance)
        if video:
            filename = f'{alg}_{self.window}_{start}_{goal}.gif'
            frames = []

        while not (start == goal):
            if self.grid_map.update(start[0], start[1]) or check:
                check = False
                result, path, nodes = alg.compute(self.grid_map, start)
                stats['Used nodes'] += nodes
                if result:
                    if video:
                        frames.append(self.grid_map.draw(start, goal, path))
                    stats['length'] += CalculateCost(start[0], start[1], path[1][0], path[1][1])
                    start = path[1]
                    path = path[1:]
                else:
                    print("Path not found!")
                    break
            else:
                if video:
                    frames.append(self.grid_map.draw(start, goal, path))
                stats['length'] += CalculateCost(start[0], start[1], path[1][0], path[1][1])
                start = path[1]
                path = path[1:]
            # print(CalculateCost(start[0], start[1], goal[0], goal[1]))
        if video:
            imageio.mimsave(filename, frames, duration=0.1)
        return stats

    def compute_tasks(self, algorithm):
        # run = wandb.init(
        #     project='PathPlanning', reinit=True
        # )
        # run.config.algorithm = str(algorithm)
        # run.config.map = self.map_name
        # run.config.window = self.window
        # run.log({'map': wandb.Image(self.grid_map.view_cells)}, step=0)
        for ind, task in enumerate(self.tasks):
            (stats), time = self.run(algorithm, (int(task[5]), int(task[4])), (int(task[7]), int(task[6])))
            stats['Time'] = time
        #     run.log(stats, ep=ind)
        # run.finish()


if __name__ == '__main__':
    os.chdir('..')
    runner = TestMapRunner('lak303d.map.scen', 3)
    # runner.run(DstarLite, (32, 4), (19, 47), True)
    print(runner.run(Astar, (18, 96), (113, 114)))
    # runner.run(DstarLite, (53, 129), (150, 119), True)
    # runner.compute_tasks(Astar)
