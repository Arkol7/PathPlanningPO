from functools import wraps
from timeit import default_timer as timer
import math


def timed(f):
    @wraps(f)
    def wrap(*args, **kw):
        start = timer()
        result = f(*args, **kw)
        elapsed = timer() - start
        return result, elapsed

    return wrap


def ManhattanDistance(i1, j1, i2, j2):
    return abs(int(i1) - int(i2)) + abs(int(j1) - int(j2))


def DiagonalDistance(i1, j1, i2, j2):
    return abs(abs(i1 - i2) - abs(j1 - j2)) + math.sqrt(2) * min(abs(i1 - i2), abs(j1 - j2))


def ChebyshevDistance(i1, j1, i2, j2):
    return max([abs(int(i1) - int(i2)), abs(int(j1) - int(j2))])


def EuclidDistance(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)


def CalculateCost(i1, j1, i2, j2):
    return math.sqrt((i1 - i2) ** 2 + (j1 - j2) ** 2)
