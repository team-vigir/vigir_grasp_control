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
//ros includes
#include "ros/ros.h"
#include <osu_grasp_msgs/Mesh_and_bounds.h>
#include "osu_grasp_msgs/CheckGraspDistance.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "plane_reps_and_3dmath.h"

#include <exception>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;
using std::exception;

class MeshStorage {
public:
	MeshStorage();

	bool check_grasp_distance(osu_grasp_msgs::CheckGraspDistance::Request& req, osu_grasp_msgs::CheckGraspDistance::Response& resp);
	void set_mesh(pcl::PolygonMesh::Ptr mesh);

	pcl::PointXYZ get_nearest_neighbor(pcl::PointXYZ point, double& distance);

	void get_preplugin_hand_pose(const geometry_msgs::PoseStamped::ConstPtr& orig_pose);
private:
	void publish_poses(geometry_msgs::PoseStamped& final_pose);
	void add_visualization(geometry_msgs::PoseStamped& hand_pose);
		
	pcl::PolygonMesh::Ptr mesh;
	pcl::KdTreeFLANN<pcl::PointXYZ> mesh_tree;
	ros::NodeHandle nh;
	ros::ServiceServer dist_service;
	geometry_msgs::PoseStamped orig_pose;
	ros::Subscriber preplugin_pose_listener;
	ros::Publisher pose_array_pub;
};


class no_search_results : public exception{
	virtual const char* what(){
		return "Could not find a nearest neighbor. Is pointcloud empty?";
	}

public:
	int ptcloud_size;
	pcl::PointXYZ pt;
};
