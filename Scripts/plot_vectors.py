import numpy as np
import mayavi.mlab as m
from ast import literal_eval

n_list = []

with open('cloud_normals.txt') as inp:
	for line in inp:
		n_list.append(literal_eval(line))

origin = [ 0, 0, 0]

X, Y, Z = zip(origin, origin, origin)
U, V, W = zip(n_list)

m.quiver3d(X, Y, Z, U, V, W)


