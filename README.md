# PathPlanningPO

Implementation of LPA* and D* Lite algorithms.

Sources:

- [Koenig, S. Likhachev, M. and Furcy, D. 2004. Lifelong Planning A*. Artificial Intelligence, 155(1-2), pp. 93-146.](http://www-cgi.cs.cmu.edu/afs/cs.cmu.edu/Web/People/maxim/files/aij04.pdf)
- [Sven Koenig and Maxim Likhachev. 2002. D*lite. In Eighteenth national conference on Artificial intelligence (AAAI 2002). 476–483.
](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
  
# Installation
- Download repository:

```bash
git clone https://github.com/Arkol7/PathPlanningPO.git
```

- Install requirements:
```bash
pip install -r requirements.txt
``` 

- Go to project directory:

```bash
cd PathPlanningPO
```

# LPA*
- run LPA* visualization on 4(or 8)-connected gridmap:
```bash
python 4-c-LPA.py
``` 
for an unknown reason, sometimes the LPA* on an 8-connected map does not find the optimal path with minimization (g + cost), so it finds the shortest path only using g-values
### Key-buttons:
- left mouse button - place cell (the first click places start cell, second one - goal cell, all other - impassable cells)
- SPACE - run LPA* from scratch
- c - clear screen
- r - re-run LPA* using information from previous run

# D* Lite

In this project 2 algorithms for solving robot navigation problem were implemented. 
The first - restarted A* (see [implementation](algorithms/Astar.py)). 
The second - D* Lite (see [implementation](algorithms/DstarLite.py)). To compare the performance 
of algorithms were created a special ``class TestMapRunner`` (see [implementation](env/runner.py)).
It uses task files and maps from [maps](maps/) directory. To evaluate performance of algorithms we calculate 4 metrics:

- Insertions - the number of node's insertions in a heap during computing.
- Accesses - the number of accesses to nodes during computing.
- Expansions - the number of expansions of nodes during computing.
- Time.

## Massive tests 
To run tests you need to have wandb account to log statistics. To give it go to their [official cite](https://wandb.ai/site).
Then you install wandb and login, you can run massive tests. To do this run:

```bash
python massive_tests.py
```

To modify test settings go to [script](massive_tests.py) and change parameters:
- ``maps`` - task files for maps. If you want to run your own map, create it according to
[MovingAI](https://movingai.com/benchmarks/grids.html) pattern, save in [maps](maps/) directory and then preprocess it with
  ``create_tests.py``. For this purpose write your filename in main function in this script and run it locally.
  ```bash
  python create_tests.py
  ```
  This script rewrites your task files, and you now can use them.
  
- ``windows`` - list of window sizes will be used.

This massive tests run all your tasks with A* and D* Lite.

## Visualization

If you want to see how agent moves on a map, use ``animation.py``.
Change arguments in``runner.run(algorithm, start, finish, True)`` on that you want.
Then you can just run:
```bash
python animation.py
```
And giff file will be generated, also script prints you some statistics.

