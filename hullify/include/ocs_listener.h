#ifndef OCS_LISTENER_H
#define OCS_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <flor_perception_msgs/RaycastRequest.h>
#include <std_msgs/Float64.h>
#include "plane_reps_and_3dmath.h"
#include <exception>

class Ocs_listener{
	public:
		Ocs_listener();

		pcl::PointXYZ get_recent_request_pt();

		void dist_request_callback(const flor_perception_msgs::RaycastRequest::ConstPtr& msg);
		void dist_result_callback(const std_msgs::Float64::ConstPtr& msg);
	private:
		ros::NodeHandle nh;
		ros::Subscriber distance_result_sub;
		ros::Subscriber distance_request_sub;

		pcl::PointXYZ* current_request_point;
		Eigen::Vector3d camera_pos;
		Eigen::Vector3d unit_camera_direction;
};

#endif
