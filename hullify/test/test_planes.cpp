#include "plane_reps_and_3dmath.h"

int main(int argc, char** argv) {
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	plane->values[0] = 1;
	plane->values[1] = 2;
	plane->values[2] = 3;
	plane->values[3] = 4;

	Eigen::Vector3d normal = get_unit_normal(plane);

	return 0;
}
