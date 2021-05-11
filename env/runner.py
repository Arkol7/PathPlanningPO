from typing import Tuple
from env.pomap import POMap
import os
from algorithms.Astar import Astar
from algorithms.DstarLite import DstarLite
from common.utils import CalculateCost, ManhattanDistance
import imageio
from common.utils import timed
import wandb
from tqdm import trange


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
        stats = {'Length': 0}
        alg = algorithm(self.grid_map, start, goal, ManhattanDistance)
        if video:
            filename = f'{alg}_{self.window}_{start}_{goal}.gif'
            frames = []

        while not (start == goal):
            self.grid_map.update(start[0], start[1])
            result, next_node = alg.compute(self.grid_map, start)
            if result:
                if video:
                    frames.append(self.grid_map.draw(start, goal))
                stats['Length'] += CalculateCost(start[0], start[1], next_node[0], next_node[1])
                start = next_node
            else:
                print("Path not found!")
                stats['Length'] = None
                break
            # print(CalculateCost(start[0], start[1], goal[0], goal[1]))
        if video:
            imageio.mimsave(filename, frames, duration=0.1)
        stats['Accesses'] = alg.accesses
        stats['Expansions'] = alg.expansions
        stats['Insertions'] = alg.insertions
        return stats

    def compute_tasks(self, algorithm):
        run = wandb.init(
            project='PathPlanningTests', reinit=True
        )
        run.config.algorithm = str(algorithm)
        run.config.map = self.map_name
        run.config.window = self.window
        run.log({'map': wandb.Image(self.grid_map.draw_initial())}, step=0)
        for ind in trange(len(self.tasks)):
            task = self.tasks[ind]
            (stats), time = self.run(algorithm, (int(task[0]), int(task[1])), (int(task[2]), int(task[3])))
            stats['Time'] = time
            stats['Optimal'] = float(task[4])
            run.log(stats, step=ind)
        run.finish()


if __name__ == '__main__':
    os.chdir('..')

    maps = ['brc505d.map.scen', 'hrt201d.map.scen', 'lak303d.map.scen']
    windows = [10, 5, 1]
    for window in windows:
        for map_name in maps:
            runner = TestMapRunner(map_name, window)
            runner.compute_tasks(DstarLite)
            runner.compute_tasks(Astar)
