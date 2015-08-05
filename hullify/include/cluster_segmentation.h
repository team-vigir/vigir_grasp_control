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
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>

#include <iostream>

#include "plane_reps_and_3dmath.h"

using std::cout;
using std::cin;
using std::endl;

struct Plane{
	pcl::ModelCoefficients::Ptr coef;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts;

};

void stat_outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, int min_cluster_size=50); //int min_cluster_size=50 was set as second parameter. not sure why.
vector<Plane> find_all_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr &full_cloud);
pcl::PointIndices::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, Plane& new_plane);

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_subcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices& indices);

Line plane_intersect(pcl::ModelCoefficients::Ptr plane1, pcl::ModelCoefficients::Ptr plane2);

void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename);
void save_planes(vector<Plane> all_planes);

pcl::PointCloud<pcl::PointXYZ>::Ptr remove_largest_plane(vector<Plane>& plane_vector);
pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud_and_planes(vector<Plane>& plane_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr return_nearest_cluster(pcl::PointXYZ selected_point, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_vector);
double return_distance_nearest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ selected_point);

//DUN DUN DUNNNNNNNNNNNNNN
pcl::PointCloud<pcl::PointXYZ>::Ptr isolate_hull_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ selected_point);

void remove_table_planes(vector<Plane>& plane_vector, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& removed_planes);
vector<int> get_all_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
double get_width(pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model, Eigen::VectorXf current_line, pcl::PointCloud<pcl::PointXYZ>::Ptr pts, pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj_pts);
double get_height(Eigen::VectorXf current_line, pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj_pts);
