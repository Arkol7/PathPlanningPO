from env.runner import TestMapRunner
from algorithms.Astar import Astar
from algorithms.DstarLite import DstarLite


maps = ['brc505d.map.scen', 'hrt201d.map.scen', 'lak303d.map.scen']
windows = [10, 5, 1]
for window in windows:
    for map_name in maps:
        runner = TestMapRunner(map_name, window)
        runner.compute_tasks(DstarLite)
        runner.compute_tasks(Astar)