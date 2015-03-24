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
	distance_request_sub = nh.subscribe("/flor/worldmodel/ocs/dist_query_distance_request_world", 1, 
						&Ocs_listener::dist_request_callback, this);
	distance_result_sub = nh.subscribe("/flor/worldmodel/ocs/dist_query_distance_result", 1, 
						&Ocs_listener::dist_result_callback, this);

	bool using_atlas;
	bool param_set = ros::param::get("/convex_hull/using_atlas", using_atlas);
	if (!param_set){
		ROS_ERROR_STREAM("Could not find parameter: /convex_hull/using_atlas in" 
				<< " ocs_listener constructor. Are you using launch files?");
		exit(1);
	}
	if (!using_atlas){
		string ptcloud_source, kinect_raw_cloud_src;
		ros::param::get("convex_hull/cloud_input_topic", ptcloud_source);
		ros::param::get("convex_hull/kinect_raw_cloud_topic", kinect_raw_cloud_src);
		ros::param::get("convex_hull/kinect_cloud_frame", kinect_cloud_frame);
		ros::param::get("convex_hull/reference_frame", world_frame);
		
		selection_point_sub = nh.subscribe(ptcloud_source, 1, 
						&Ocs_listener::kinect_pt_select_callback, this);
		kinect_raw_cloud_sub = nh.subscribe(kinect_raw_cloud_src, 1, 
						&Ocs_listener::kinect_raw_cloud_callback, this);
		box_selection_pub = nh.advertise<sensor_msgs::PointCloud2>("/selected_points", 1);
	}

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
	if (msg->data < 0){
		set_request_point(NULL);

	} else {
		current_request_point = new pcl::PointXYZ;
		Eigen::Vector3d dist = unit_camera_direction * msg->data;
		*current_request_point = init_pt(camera_pos + dist);
	}
}

void Ocs_listener::kinect_pt_select_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	int num_pts = msg->width * msg->height;
	//ROS_INFO("Got a cloud for ocs_listener");
	if (num_pts > 0 && num_pts < 5){
		ROS_INFO("Got a selection point from the kinect.");
		sensor_msgs::PointCloud2 temp_cloud = *msg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::moveFromROSMsg(temp_cloud, *cloud);
		set_request_point(& ((*cloud)[0]));

		publish_box_selection();
	}
}

void Ocs_listener::kinect_raw_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	current_raw_cloud = msg;
}

void Ocs_listener::publish_box_selection()
{
	string original_ptcloud_frame = current_raw_cloud->header.frame_id;
	sensor_msgs::PointCloud2 recent_cloud = *current_raw_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
						temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::moveFromROSMsg(recent_cloud, *temp_cloud);
	pcl_ros::transformPointCloud(world_frame, *temp_cloud, *cloud, listener);

	ROS_INFO_STREAM("Input to box selection has " << cloud->size() << " points.");
	pcl::PointCloud<pcl::PointXYZ>::Ptr selected_cloud = select_box(cloud, 0.3);
	ROS_INFO_STREAM("Output of box selection has " << selected_cloud->size() << " points.");

	ROS_INFO_STREAM("Original Pointcloud frame: " << original_ptcloud_frame);
	pcl_ros::transformPointCloud(original_ptcloud_frame, *selected_cloud, *temp_cloud, listener);
	sensor_msgs::PointCloud2 out_cloud;
	pcl::toROSMsg(*temp_cloud, out_cloud);
	
	box_selection_pub.publish(out_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Ocs_listener::select_box(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, double width)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	out_cloud->header = in_cloud->header;
	//pcl::PointXYZ transformed_select_point = transform_request_point(); //Necessary if the cloud isn't in the world frame
	pcl::PointXYZ transformed_select_point = *current_request_point;
	cout << "Transformed point: " << transformed_select_point.x << "  " << transformed_select_point.y << "  " << transformed_select_point.z << endl;
	long num_pts = in_cloud->size();
	for (long i = 0; i < num_pts; ++i){
		if (fabs((*in_cloud)[i].x - transformed_select_point.x) < (width/2) &&
			fabs((*in_cloud)[i].y - transformed_select_point.y) < (width/2) &&
			fabs((*in_cloud)[i].z - transformed_select_point.z) < (width/2)) {
			out_cloud->push_back((*in_cloud)[i]);
		}
	}

	return out_cloud;
}

pcl::PointXYZ Ocs_listener::transform_request_point()
{
	tf::StampedTransform transform;
        tf::Vector3 temp_vec(current_request_point->x, current_request_point->y, current_request_point->z);
	while (1){
		try {
			listener.lookupTransform(kinect_cloud_frame, world_frame,
	                                          ros::Time(0), transform);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
	 		sleep(1);
			continue;
	 	}
	 
	 	break;
	 }

	tf::Vector3 transformed_point = transform(temp_vec);
	return init_pt(transformed_point[0], transformed_point[1], transformed_point[2]);
}

void Ocs_listener::set_request_point(pcl::PointXYZ* pt)
{
	if (pt == NULL && current_request_point != NULL){
		delete current_request_point;
		current_request_point = NULL;
		return;

	} else if (pt == NULL){
		return;
	}
	
	if (current_request_point == NULL){
		current_request_point = new pcl::PointXYZ;
	}

	*current_request_point = *pt;
}

pcl::PointXYZ Ocs_listener::get_recent_request_pt()
{
	if (current_request_point != NULL){
		return *current_request_point;

	} else {
		throw 1;
	}
}
