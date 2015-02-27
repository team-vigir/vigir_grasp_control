from numpy import *
import plane_filters

if __name__ == "__main__":
	print "Testing removing rollless approaches"
	approaches = array([3,4,5,6])
	wrist_orientations = [[1], [], [], [1]]

	print "wrist_orientations (Pre): ", wrist_orientations
	print "aproaches (Pre): ", approaches, "\n"

	new_approaches = plane_filters.remove_rolless_approaches(wrist_orientations, approaches)

	print "wrist_orientations (Post): ", wrist_orientations
	print "approaches (post): ", new_approaches