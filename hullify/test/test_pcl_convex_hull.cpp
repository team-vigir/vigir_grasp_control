/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
