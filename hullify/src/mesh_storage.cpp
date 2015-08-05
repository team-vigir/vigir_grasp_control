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
#include "mesh_storage.h"

MeshStorage::MeshStorage()
{
	//Register the service callback
	dist_service = nh.advertiseService("CheckGraspDistance", &MeshStorage::check_grasp_distance, this);
	preplugin_pose_listener = nh.subscribe("openrave_preplugin_grasp", 1, &MeshStorage::get_preplugin_hand_pose, this);
	pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("openrave_dist_check", 1);
}

void MeshStorage::set_mesh(pcl::PolygonMesh::Ptr mesh)
{
	this->mesh = mesh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(mesh->cloud, *cloud);
	mesh_tree.setInputCloud(cloud);
}

bool MeshStorage::check_grasp_distance(osu_grasp_msgs::CheckGraspDistance::Request& req, osu_grasp_msgs::CheckGraspDistance::Response& resp)
{
	//add_visualization(req.hand_pose);
	
	pcl::PointXYZ wrist_center_pt;
	wrist_center_pt.x = req.hand_pose.pose.position.x;
	wrist_center_pt.y = req.hand_pose.pose.position.y;
	wrist_center_pt.z = req.hand_pose.pose.position.z;

	double nearest_distance, projected_dist;
	try {
		pcl::PointXYZ nearest_neighbor = get_nearest_neighbor(wrist_center_pt, nearest_distance);
		Eigen::Vector3d palm_approach(0, 1, 0);
		Eigen::Vector3d hand_center_pt = init_vec( wrist_center_pt);

		Eigen::Quaterniond hand_rot(req.hand_pose.pose.orientation.w,req.hand_pose.pose.orientation.x, req.hand_pose.pose.orientation.y, req.hand_pose.pose.orientation.z);
		palm_approach = hand_rot._transformVector(palm_approach);

		palm_approach.normalize();
		Eigen::ParametrizedLine<double, 3> line(hand_center_pt, palm_approach);
		Eigen::Vector3d proj_pt = line.projection(init_vec(nearest_neighbor)) - hand_center_pt;
		projected_dist = proj_pt.norm();
	
	} catch (exception& e) {
		cout << e.what() << endl;
		resp.within_acceptable_dist = false;
		return false;
	}

/*	cout << "nearest_dist: " << nearest_distance << " projected dist: " << projected_dist << "acceptable dist: " << req.acceptable_dist << endl;
*/
	if (projected_dist <= req.acceptable_dist){
		//cout << "\tGrasp good!" << endl;
		resp.within_acceptable_dist = true;
	} else {
		//cout << "\tGrasp rejected" << endl;
		resp.within_acceptable_dist = false;
	}

	return true;
}

pcl::PointXYZ MeshStorage::get_nearest_neighbor(pcl::PointXYZ point, double& distance)
{
	vector<int> nearest_neighbor_idx(1);
	vector<float> nearest_neighbor_dist_sq(1);

	if ( mesh_tree.nearestKSearch (point, 1, nearest_neighbor_idx, nearest_neighbor_dist_sq) > 0 ){
		distance = sqrt(nearest_neighbor_dist_sq[0]);
		return mesh_tree.getInputCloud()->points[nearest_neighbor_idx[0]];

	} else {
		no_search_results excp;
		excp.ptcloud_size = mesh_tree.getInputCloud()->points.size();
		excp.pt = point;
		throw no_search_results();
	}
}

void MeshStorage::add_visualization(geometry_msgs::PoseStamped& hand_pose)
{
	publish_poses(hand_pose);
	cout << "How do those poses look?" << endl;
	string input;
	cin >> input;
}

void MeshStorage::get_preplugin_hand_pose(const geometry_msgs::PoseStamped::ConstPtr& orig_pose)
{
	this->orig_pose = *orig_pose;
}

void MeshStorage::publish_poses(geometry_msgs::PoseStamped& final_pose)
{
	geometry_msgs::PoseArray both_poses;
	both_poses.header = orig_pose.header;
	both_poses.poses.push_back(orig_pose.pose);
	both_poses.poses.push_back(final_pose.pose);

	pose_array_pub.publish(both_poses);
}
