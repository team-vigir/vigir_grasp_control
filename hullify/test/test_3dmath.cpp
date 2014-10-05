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

int main(int argc, char** argv){
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}