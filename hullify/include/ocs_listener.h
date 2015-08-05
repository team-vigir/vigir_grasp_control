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
#ifndef OCS_LISTENER_H
#define OCS_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vigir_perception_msgs/RaycastRequest.h>
#include <std_msgs/Float64.h>
#include "plane_reps_and_3dmath.h"
#include <exception>

#include <tf/transform_listener.h>

#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>

class Ocs_listener{
	public:
		Ocs_listener();

		pcl::PointXYZ get_recent_request_pt();

		void dist_request_callback(const vigir_perception_msgs::RaycastRequest::ConstPtr& msg);
		void dist_result_callback(const std_msgs::Float64::ConstPtr& msg);
		void kinect_pt_select_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
		void kinect_raw_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

	private:
		void set_request_point(pcl::PointXYZ* pt);
		void publish_box_selection();
		pcl::PointCloud<pcl::PointXYZ>::Ptr select_box(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, double width);
		pcl::PointXYZ transform_request_point();

		ros::NodeHandle nh;
		ros::Subscriber distance_result_sub;
		ros::Subscriber distance_request_sub;
		ros::Subscriber selection_point_sub;
		ros::Subscriber kinect_raw_cloud_sub;
		ros::Publisher box_selection_pub;

		tf::TransformListener listener;
		string kinect_cloud_frame;
		string world_frame;
	
		sensor_msgs::PointCloud2::ConstPtr current_raw_cloud;
		pcl::PointXYZ* current_request_point;
		Eigen::Vector3d camera_pos;
		Eigen::Vector3d unit_camera_direction;
};

#endif
