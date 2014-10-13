#include "plane_reps_and_3dmath.h"
#include "cluster_segmentation.h"
#include "gtest/gtest.h"

pcl::ModelCoefficients::Ptr mk_xy_plane()
{
	pcl::ModelCoefficients::Ptr xy_plane (new pcl::ModelCoefficients);
	xy_plane->values.resize(4);
	xy_plane->values[0] = 0;
	xy_plane->values[1] = 0;
	xy_plane->values[2] = 1;
	xy_plane->values[4] = 0;

	return xy_plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr mk_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  return cloud;
}

TEST(get_normal, simple_test){
	pcl::ModelCoefficients::Ptr plane = init_plane(1, 2, 3, 4);

	/** Forrest: this is your test, it was seg faulting because the coefficient vector for the plane was not sized properly. no resize() **/

	Eigen::Vector3d normal = get_unit_normal(plane);
	EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0.26726124, 0.53452248, 0.801783725), normal, 0.01));
}

TEST(line_eval, null_slope){
	int num_lines = 3;
	Line** lines = new Line*[num_lines];
	lines[0] = new Line();
	lines[1] = new Line(Eigen::Vector3d(0, 0, 0.0001), Eigen::Vector3d(0, 0, 1));
	lines[2] = new Line(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0.0001));
	lines[2]->slope -= lines[2]->slope;


	for (int i = 0; i < num_lines; ++i){
		try {
			lines[i]->get_pt(0);
			cout << "Actually got a point..." << endl;
			EXPECT_TRUE(false);

		} catch (exception& e){
			cout << e.what() << endl;
			EXPECT_TRUE(true);

		} catch (...) {
			cout << "Caught unexpected exception." << endl;
			EXPECT_TRUE(false);
		}
	}
}

TEST(line_eval, normal_plane_intersection){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	pcl::ModelCoefficients::Ptr rand_plane = init_plane(1, 2, 3, -1);

	Line l1(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 2, 3));
	Line l2(Eigen::Vector3d(1, 2, 2), Eigen::Vector3d(0, 0, 0));
	Eigen::Vector3d res;

	try {
		EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(1, 2, 0), l1.find_plane_intersection(xy_plane), 0.01));

		res = l2.find_plane_intersection(xy_plane);
		cout << "Res: " << endl << res << endl;
		EXPECT_TRUE(is_null_vec(res));

		res = l1.find_plane_intersection(rand_plane);
		cout << "Res: " << endl << res << endl;
		EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(1, 2, -1.333333), res, 0.01));
		
		res = l2.find_plane_intersection(rand_plane);
		cout << "Res: " << endl << res << endl;
		EXPECT_TRUE(vecs_are_equal(Eigen::Vector3d(0.090909, 0.1818181818, 0.181818181818), res, 0.01));
	
	} catch (exception& e) {
		cout << e.what() << endl;
		EXPECT_TRUE(false);
	}
}

TEST(line_eval, no_plane_intersection){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	pcl::ModelCoefficients::Ptr rand_plane = init_plane(0.5, -1, 2, 2);

	Line l1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 1));
	Line l2(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(5, -12, 0.2));
	Line l3(Eigen::Vector3d(0, 2, 1), Eigen::Vector3d(0, 0, 0));

	try { l1.find_plane_intersection(xy_plane); EXPECT_TRUE(false); } catch (exception& e) { EXPECT_TRUE(true);} catch (...){cout << "Unexpected exception."; EXPECT_TRUE(false);}
	try { l2.find_plane_intersection(xy_plane); EXPECT_TRUE(false);} catch (exception& e) { EXPECT_TRUE(true);} catch (...){cout << "Unexpected exception."; EXPECT_TRUE(false);}
	try { l3.find_plane_intersection(rand_plane); EXPECT_TRUE(false);} catch (exception& e) { EXPECT_TRUE(true);} catch (...){cout << "Unexpected exception."; EXPECT_TRUE(false);}
}

TEST(line_eval, line_in_plane){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	Line l1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(1, 1, 0));

	try{
		l1.find_plane_intersection(xy_plane);
		EXPECT_TRUE(false);

	} catch (exception& e){
		EXPECT_TRUE(true);
	} catch (...) {
		cout << "Caught Unexpected exception." << endl;
		EXPECT_TRUE(false);
	}
}

TEST(plane_intersection, first_times_the_charm){
	pcl::ModelCoefficients::Ptr xy_plane = mk_xy_plane();
	pcl::ModelCoefficients::Ptr yz_plane = init_plane(1, 0, 0, 0);

	Line intersection = plane_intersect(xy_plane, yz_plane);
	cout << "Slope: " << intersection.slope << endl 
		 << "Point: " << intersection.intercept << endl;

	EXPECT_TRUE(true);
}

TEST(plane_intersection, first_times_the_charm_strikes_again){
	pcl::ModelCoefficients::Ptr xy_plane = init_plane(65, 5, -8, 72);
	pcl::ModelCoefficients::Ptr yz_plane = init_plane(5, 5, 5, -100);

	Line intersection = plane_intersect(xy_plane, yz_plane);
	cout << "Slope: " << intersection.slope << endl 
		 << "Point: " << intersection.intercept << endl;

	EXPECT_TRUE(true);
}

TEST(plane_segmentation, hope_it_works){
	
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud = mk_cloud(cloud);
*/  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 	Plane plane;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_1.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }

	planar_segmentation(cloud, plane);

	EXPECT_TRUE(true);
}

/*TEST(plane_segmentation, it_will_work){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 	
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }
    stat_outlier_remove(cloud);
    vector<Plane> all_planes = find_all_planes(cloud);

    save_planes(all_planes);
    save_cloud(cloud, "remaining_cloud");

    EXPECT_TRUE(true);
}*/

/*TEST(remove_plane, woo){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 	
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }
    stat_outlier_remove(cloud);
    vector<Plane> all_planes = find_all_planes(cloud);
    remove_largest_plane(all_planes);
    save_planes(all_planes);
    save_cloud(cloud, "remaining_cloud");

    EXPECT_TRUE(true);
}*/

TEST(combine_pcs, anything){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
 	
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }
    stat_outlier_remove(cloud);
    vector<Plane> all_planes = find_all_planes(cloud);
    remove_largest_plane(all_planes);
   	combined_cloud = combine_cloud_and_planes(all_planes, cloud);
   	save_cloud(combined_cloud, "combined_cloud");

    EXPECT_TRUE(true);
}

TEST(clusterify, easy){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud; 

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }
    stat_outlier_remove(cloud);
    vector<Plane> all_planes = find_all_planes(cloud);
    remove_largest_plane(all_planes);
   	combined_cloud = combine_cloud_and_planes(all_planes, cloud);

   	pcl::PointXYZ point;
   	get_clusters(combined_cloud, point);

   	EXPECT_TRUE(true);
}

TEST(nearest_cluster, ninety_nine_percent_completion_mark) {
	pcl::PointXYZ rand_point = init_pt(-0.33621544, -0.7632941, 0.85465014);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud; 

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }
    stat_outlier_remove(cloud);
    vector<Plane> all_planes = find_all_planes(cloud);
    remove_largest_plane(all_planes);
   	combined_cloud = combine_cloud_and_planes(all_planes, cloud);

   	pcl::PointXYZ point;
   	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters = get_clusters(combined_cloud, point);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = return_nearest_cluster(rand_point, clusters);
   	save_cloud(cluster, "isolated_cloud");

   	EXPECT_TRUE(true);
}

int main(int argc, char** argv){
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}