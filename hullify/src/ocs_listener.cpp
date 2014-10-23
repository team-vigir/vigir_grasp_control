#include "ocs_listener.h"

/*
int main(int argc, char** argv){
	ros::init(argc, argv, "ptcloud_request_ROI_pt_test");
	Ocs_listener the_ninja;

	ros::spin();

	return 0;
}
*/

Ocs_listener::Ocs_listener()
{
	distance_request_sub = nh.subscribe("/flor/worldmodel/ocs/dist_query_distance_request_world", 1, &Ocs_listener::dist_request_callback, this);
	distance_result_sub = nh.subscribe("/flor/worldmodel/ocs/dist_query_distance_result", 1, &Ocs_listener::dist_result_callback, this);

	current_request_point = NULL;
}

void Ocs_listener::dist_request_callback(const flor_perception_msgs::RaycastRequest::ConstPtr& msg)
{
	//cout << "Request called!" << endl;
	camera_pos = Eigen::Vector3d(msg->origin.x, msg->origin.y, msg->origin.z);
	unit_camera_direction = Eigen::Vector3d(msg->direction.x, msg->direction.y, msg->direction.z);

	unit_camera_direction.normalize();
}


void Ocs_listener::dist_result_callback(const std_msgs::Float64::ConstPtr& msg)
{
	if (current_request_point != NULL){
		delete current_request_point;
	}

	if (msg->data < 0){
		//cout << "No result found" << endl;
		current_request_point = NULL;

	} else {
		current_request_point = new pcl::PointXYZ;
		Eigen::Vector3d dist = unit_camera_direction * msg->data;
		//cout << "dist: " << dist << endl << "camera_pos: " << camera_pos << endl;
		*current_request_point = init_pt(camera_pos + dist);
		//cout << "result pt: x-" << current_request_point->x << " y-" << current_request_point->y << " z-" << current_request_point->z << endl;
	}
}

pcl::PointXYZ Ocs_listener::get_recent_request_pt()
{
	if (current_request_point != NULL){
		return *current_request_point;

	} else {
		throw 1;
	}
}
