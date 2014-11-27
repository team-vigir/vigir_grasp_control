/***************************************
* File: plane_reps_and_3dmath.h
* Description: This code handles creating planes in
*	3d space from vectors and points along with other 
*	necessary mathematical functionality (distance between
*	two points and projections)
* Author: Jackson Carter
* Date: 7/10/2014
****************************************/
#ifndef PLANE_REPS_AND_3DMATH_H
#define PLANE_REPS_AND_3DMATH_H

//ros includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>

#include <Eigen/Dense>

#include <exception>
#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;
using std::exception;

#define FLOAT_TOLERANCE 0.001

struct Pt_Dist {
	double dist;
	long idx;
};

class Line {
public:
	Line();
	Line(Eigen::Vector3d slope, Eigen::Vector3d intercept);

	Eigen::Vector3d get_pt(double t);
	Eigen::Vector3d find_plane_intersection(pcl::ModelCoefficients::Ptr plane);
	pcl::PointXYZ project_pcl(pcl::PointXYZ pt);
	Eigen::Vector3d project(pcl::PointXYZ pt);

	Eigen::Vector3d slope;
	Eigen::Vector3d intercept;
};

//Function Declarations
double difference(double var1, double var2);
int dist_compare(const void* a, const void* b);

Eigen::Vector3d init_vec(const pcl::PointXYZ& in);
Eigen::Vector3d init_vec(const pcl::PointXYZ& final, const pcl::PointXYZ& initial);
bool vecs_are_equal(Eigen::Vector3d v1, Eigen::Vector3d v2, double custom_tolerance=FLOAT_TOLERANCE);
bool is_null_vec(const Eigen::Vector3d& vec, double custom_tolerance=FLOAT_TOLERANCE);
double get_angle_mag_between(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

pcl::PointXYZ init_pt(double x, double y, double z);
pcl::PointXYZ init_pt(Eigen::Vector3d vec);
double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2);
pcl::PointXYZ find_farthest_pt(pcl::PointXYZ location, pcl::PointCloud<pcl::PointXYZ>::Ptr other_points);

pcl::ModelCoefficients::Ptr init_plane(double a, double b, double c, double d);
void normalize_plane(pcl::ModelCoefficients* plane);
void normalize_plane(pcl::ModelCoefficients::Ptr plane);
pcl::ModelCoefficients::Ptr mk_plane(Eigen::Vector3d pt, Eigen::Vector3d normal);
bool planes_are_eq(pcl::ModelCoefficients::Ptr in_plane1, pcl::ModelCoefficients::Ptr in_plane2);
pcl::PointCloud<pcl::PointXYZ>::Ptr project_pts_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::ModelCoefficients::Ptr plane);
Eigen::Vector3d vec_to_pt_on_plane(pcl::ModelCoefficients::Ptr plane);
double pt_to_plane_dist(pcl::ModelCoefficients::Ptr hess_norm_plane, const pcl::PointXYZ& pt);
void find_max_min_dist_from_plane(pcl::ModelCoefficients::Ptr hess_norm_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr pts, int* min_max);
void print_max_min_distances(Pt_Dist* dist_arr, long num_pts);

void mk_sandwich_planes(pcl::ModelCoefficients::Ptr orig_plane, double dist_orig_to_new, pcl::ModelCoefficients::Ptr* out_planes);
pcl::PointCloud<pcl::PointXYZ>::Ptr find_pts_between_parallel_planes(pcl::ModelCoefficients::Ptr* ordered_planes, pcl::PointCloud<pcl::PointXYZ>::Ptr pts);
	bool pt_below_plane(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ& pt);
	bool pt_above_plane(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ& pt);

//STUFF NOT JACKSON WRITE
Eigen::Vector3d get_unit_normal(pcl::ModelCoefficients::Ptr plane);
Eigen::Vector3d get_normal(pcl::ModelCoefficients::Ptr plane);

class plane_line_exc : public exception
{
  virtual const char* what() const throw()
  {
    return "The line never touches the plane, or it runs on the plane.";
  }
};

class null_slope : public exception
{
  virtual const char* what() const throw()
  {
    return "The line has a null slope. It was improperly initialized.";
  }
};

#endif