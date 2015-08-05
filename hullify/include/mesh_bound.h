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
/***************************************************************************
* File: mesh_bound.h/.cpp
* Description: To decompose the making of an object from the
*   identification of its center and bounding planes, this object
*   was created. It requires the original pointcloud and can determine
*   all centroid and plane information.
* Notes: Centroid and plane data can be queried via getters; they are not
*   returned via methods.
***************************************************************************/
#ifndef MESH_BOUND_H
#define MESH_BOUND_H

//ros includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>

#include <tf/transform_listener.h>

#include "pcl/ros/conversions.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <Eigen/Dense>

#include "plane_reps_and_3dmath.h"
#include "hullify_view.h"

#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;

//See find_max_radial_dist()
struct Pt_pos {
	long idx;	//Index into pointcloud for a point
	double angle;	//The point's radial position from reference
};

inline Eigen::Vector3d init_vec(const tf::Vector3& in);
int angle_compare(const void* a, const void* b);

class MeshBound {
	public:
		MeshBound();
		MeshBound(string ff, Hullify_View* in_view);
		MeshBound(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, string ff, Hullify_View* in_view);

		void set_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);
		void display_polygon(geometry_msgs::PolygonStamped polygon);
		void construct_planes();

		pcl::ModelCoefficients get_plane1();
		Eigen::Vector3d get_plane1_normal();
		pcl::ModelCoefficients get_plane2();
		Eigen::Vector3d get_plane2_normal();
		Eigen::Vector3d get_centroid();
		Eigen::Vector3d get_camera_normal_vec();
		Eigen::Vector3d get_horiz_normal();

		void publish_centroid();
		void publish_plane1();
		void publish_plane2();
		geometry_msgs::PolygonStamped mk_plane_msg_ext(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ center);
		Eigen::Vector3d get_torso_normal_vec(string arm);
		
	private:
		void constructor_common();
		void init_perception_link();
		void init_torso_link();
		void find_centroid();
		void get_camera_normal();
		void find_horiz_normal();
		geometry_msgs::PolygonStamped mk_plane_msg(pcl::ModelCoefficients::Ptr plane);
		Eigen::Vector3d get_camera_position();
		pcl::PointCloud<pcl::PointXYZ>::Ptr extract_middle_points(pcl::ModelCoefficients::Ptr horiz_plane);
		int* get_max_radial_dist(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		int* get_max_radial_dist_core(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, Eigen::Vector3d& ref_vec, Eigen::Vector3d& line_normal);
		Pt_pos* find_all_pt_angles(Eigen::Vector3d& ref_line_slope, Eigen::Vector3d& ref_line_orth, pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts, long& num_pts);
		bool pt_too_near_centroid(Pt_pos* pt_angles, long& pt_idx, Eigen::Vector3d& pt_to_centroid, long& num_pts);
		void print_pt_angles(Pt_pos* pt_angles, long num_pts);
		Eigen::Vector3d* get_ref_line_slope(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);
		bool pt_wraps_past_ref_line(const Eigen::Vector3d& line_norm, pcl::PointXYZ& pt);
		void calculate_parametric_coefficients_for_proj(int coord1, int coord2, Eigen::Vector3d slope, Eigen::Vector3d line_norm, Eigen::Vector3d pt, double& s, double& t);
		int* find_max_consecutive_angular_diff(Pt_pos* pt_angles, long num_pts);
		void standardize_plane_normals(Eigen::Vector3d& n1, Eigen::Vector3d& n2);

		void handle_ptcloud_errors();		

		void publish_proj_pts(pcl::PointCloud<pcl::PointXYZ>::Ptr proj_pts);


		ros::NodeHandle n;
		tf::TransformListener listener;
		string fixed_frame;
		string perception_link;
		string torso_link;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		Eigen::Vector3d *centroid;
		Eigen::Vector3d camera_normal;	//This is a unit vector
		Eigen::Vector3d horiz_normal;
		pcl::ModelCoefficients::Ptr plane1;
		pcl::ModelCoefficients::Ptr plane2;

		Hullify_View* view;
};

class tooManyPtsNearCentroid {
public:
	tooManyPtsNearCentroid();
	tooManyPtsNearCentroid(int leftover_pt_count);

	void print_error();
	int leftover_pt_count;
};

class insufficientPoints {
public:
	insufficientPoints();
	insufficientPoints(int num_pts);

	void print_error();
	int num_pts;
};

class noRefVec {
public:
	noRefVec();
	void print_error();
};

#endif
