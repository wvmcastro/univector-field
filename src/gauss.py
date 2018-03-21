from math import exp

def gaussian(m, v):
    return exp(-(m**2) / (2 * (v**2)))