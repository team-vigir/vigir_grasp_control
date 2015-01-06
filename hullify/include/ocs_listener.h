#ifndef OCS_LISTENER_H
#define OCS_LISTENER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <flor_perception_msgs/RaycastRequest.h>
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

		void dist_request_callback(const flor_perception_msgs::RaycastRequest::ConstPtr& msg);
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
