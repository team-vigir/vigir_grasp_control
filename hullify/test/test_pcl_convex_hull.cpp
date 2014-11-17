#include "gtest/gtest.h"
#include <sys/time.h>
#include <iostream>
#include "mesh_maker.h"


using std::cout;
using std::endl;

pcl::PointCloud<pcl::PointXYZ>::Ptr mk_cloud(long size)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width  = size;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	// Generate the data
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		  cloud->points[i].x = (rand() % 100) / (float) 50;
		  cloud->points[i].y = (rand() % 100) / (float) 50;
		  cloud->points[i].z = (rand() % 100) / (float) 50;
	}

	return cloud;
}

TEST(convex_hull_creation, timing_test){
	//Make the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mk_cloud(750);
	MeshMaker meshy;

	struct timeval start, end;
	gettimeofday(&start, NULL);

	meshy.mk_mesh(cloud);

	gettimeofday(&end, NULL);
	cout << "Running PCL's convex hull algorithm on "<< cloud->points.size()<< " took approximately " << end.tv_sec - start.tv_sec << "s " << end.tv_usec - start.tv_usec << "us." << endl;

	EXPECT_EQ(true, true);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_chull_testing");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
