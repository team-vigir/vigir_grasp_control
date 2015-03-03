import numpy as np

def unit_vector(v):
	unit_vec = v / np.linalg.norm(v)
	return unit_vec

def angle_between(v1, v2):
	v1_unit = unit_vector(v1)
	v2_unit = unit_vector(v2)

	angle = np.arccos(np.dot(v1_unit, v2_unit))
	if np.isnan(angle):
		if (v1_unit == v2_unit).all():
			return 0
		else:
			return np.pi
	return angle
