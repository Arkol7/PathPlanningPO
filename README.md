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

# Introduction

# LPA*
- run LPA* visualization on 4(or 8)-connected gridmap:
```bash
python 4-c-LPA.py
``` 
for an unknown reason, sometimes the lpa on an 8-connected map does not find optimal path with minimization (g + cost), so it finds shortest path only using g-values
### Key-buttons:
- left mouse button - place cell (the first click places start cell, second one - goal cell, all other - impassable cells)
- SPACE - run LPA* from scratch
- c - clear screen
- r - re-run LPA* using information from previous run

# D* Lite
