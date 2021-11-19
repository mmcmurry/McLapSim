# Some helper functions that are used elsewhere in the Lap Sim. 

from math import sqrt

def quadraticFormula(a,b,c):
    # Given coefficients of a quadratic, finds x-coordinates of the zeroes and returns them in a list.
    result = []
    result.append((-b+sqrt(b**2-4*a*c))/(2*a))
    result.append((-b-sqrt(b**2-4*a*c))/(2*a))
    return result

def saturate(x, lb, ub):
    # Equivalent to Simulink saturation block. Bounds x between upper and lower bounds lb and ub.
    return max(lb, min(ub, x))