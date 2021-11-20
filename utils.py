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

def interp2(x, y, xq, extrap):
    # Interpolate through table f(x)=y at query point xq. Can extrapolate if out of bounds of table or clip.
    length = len(x)

    for i in range(length-1):
        if xq > x[i] and xq <= x[i+1]:
            return y[i] + (y[i+1] - y[i]) / (x[i+1] - x[i]) * (xq - x[i])

    if extrap:
        # Linear extrapolation
        if xq > max(x):
            return y[length-1] + (y[length-1] - y[length-2]) / (x[length-1] - x[length-2]) * (xq - x[length-1])
        else:
            return y[0] - (y[1] - y[0]) / (x[1] - x[0]) * (x[0] - xq)
    else:
        # Clipping
        if xq > max(x):
            return y[length-1]
        else:
            return y[0]

def readTrackFile(path):
		# Turns a .txt file into a track. Works when you copy/paste radius/angle/distance data from Excel into Notepad
		file = open(path)
		track = []
		
		for line in file:
			turn = line.split('\t')
			for i in range(3):
				turn[i] = float(turn[i])
			track.append(turn)
			
		file.close()
		return track