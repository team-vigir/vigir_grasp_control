#include "mesh_maker.h"

int main (int argc, char** argv){	
	//initalize the node
	ros::init(argc, argv, "convex_hull");

	MeshMaker converter;
	converter.listen();

	return 0;
}
