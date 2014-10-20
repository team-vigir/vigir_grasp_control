#include "mesh_storage.h"
#include "plane_reps_and_3dmath.h"
#include "gtest/gtest.h"

#include <iostream>
using std::cout;
using std::cin;
using std::endl;

/*Axes mk_simple_axes()
{
	Axes simple_axes;
	simple_axes.x_axis = Eigen::Vector3d(1, 0, 0);
	simple_axes.y_axis = Eigen::Vector3d(0, 1, 0);
	simple_axes.z_axis = Eigen::Vector3d(0, 0, 1);

	return simple_axes;	
}*/

pcl::PointCloud<pcl::PointXYZ>::Ptr mk_simple_cloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PointXYZ pt = init_pt(0, 0, 0);
	out_cloud->push_back(pt);

	pt = init_pt(1, 0, 0);
	out_cloud->push_back(pt);

	pt = init_pt(0, 1, 0);
	out_cloud->push_back(pt);

	pt = init_pt(0, 0, 1);
	out_cloud->push_back(pt);

	pt = init_pt(1, 1, 0);
	out_cloud->push_back(pt);

	return out_cloud;
}

pcl::PolygonMesh::Ptr mk_cloud_only_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
	toPCLPointCloud2(*cloud, mesh->cloud);

	return mesh;
}

geometry_msgs::PoseStamped mk_pose()
{
	geometry_msgs::PoseStamped hand_pose;
	hand_pose.pose.position.z = -1;
	hand_pose.pose.orientation.w = 1;

	return hand_pose;
}

TEST(mesh_storage_nearest_neighbor, simple_tests){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mk_simple_cloud();
	pcl::PolygonMesh::Ptr mesh = mk_cloud_only_mesh(cloud);
	double distance = 100;
	MeshStorage dist_checker;
	dist_checker.set_mesh(mesh);

	pcl::PointXYZ search_pt = init_pt(0, 0, 0);
	pcl::PointXYZ closest_pt = dist_checker.get_nearest_neighbor(search_pt, distance);
	EXPECT_EQ(0, distance);

	search_pt = init_pt(1, 0, 0);
	closest_pt = dist_checker.get_nearest_neighbor(search_pt, distance);
	EXPECT_EQ(0, distance);
	EXPECT_EQ(1, closest_pt.x);

	search_pt = init_pt(2, 0, 0);
	closest_pt = dist_checker.get_nearest_neighbor(search_pt, distance);
	EXPECT_EQ(1, distance);
	EXPECT_EQ(1, closest_pt.x);

	search_pt = init_pt(1, 1, 1);
	closest_pt = dist_checker.get_nearest_neighbor(search_pt, distance);
	EXPECT_EQ(1, distance);
	EXPECT_EQ(1, closest_pt.x);
	EXPECT_EQ(1, closest_pt.y);
}

TEST(full_check_dist_pipeline, simple_tests){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = mk_simple_cloud();
	pcl::PolygonMesh::Ptr mesh = mk_cloud_only_mesh(cloud);
	MeshStorage dist_checker;
	dist_checker.set_mesh(mesh);

	geometry_msgs::PoseStamped hand_pose = mk_pose();
	osu_grasp_msgs::CheckGraspDistance::Request req;
	osu_grasp_msgs::CheckGraspDistance::Response resp;
	req.hand_pose = hand_pose;
	req.acceptable_dist = 0.5;
	dist_checker.check_grasp_distance(req, resp);
	cout << "The check returned " << resp.within_acceptable_dist << " for the pose beneath the object." << endl;
	EXPECT_EQ(false, resp.within_acceptable_dist);

	req.hand_pose.pose.position.y = -1;
	req.hand_pose.pose.position.z = 0;
	dist_checker.check_grasp_distance(req, resp);
	EXPECT_EQ(false, resp.within_acceptable_dist);

	req.acceptable_dist = 1.01;
	dist_checker.check_grasp_distance(req, resp);
	EXPECT_EQ(true, resp.within_acceptable_dist);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "storage_tests");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
