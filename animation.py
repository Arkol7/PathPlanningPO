from env.runner import TestMapRunner
from algorithms.Astar import Astar
from algorithms.DstarLite import DstarLite

map_name = 'lak303d.map.scen'
window = 5

runner = TestMapRunner(map_name, window)
print(runner.run(Astar, (140, 40), (90, 90), True))
