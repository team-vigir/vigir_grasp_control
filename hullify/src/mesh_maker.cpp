#include "mesh_maker.h"

//Ought to work. Untested because qhull libraries were insufficient.
//  Requires precisely Qhulllib5 (2011.1)
pcl::PolygonMesh::Ptr MeshMaker::mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	//See PCL Concave Hull tutorial for filters
	//cout << "No filters for pointcloud. Is it filtered?" << endl;
	
	pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);

	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(cloud);
	//hull.reconstruct(*output);

	output->header.stamp = 1;
	output->header.frame_id = visualization_ref_frame;

	// Finish
	return output;
}


MeshMaker::MeshMaker()
: listener(n)
{
	visualization_ref_frame = "/world";
	mesh_ref_frame = "/pelvis";
	using_left_hand = true;
	//cout << "Hi Jack!" << endl;
	cout << "Currently, this program has minimal validation. Please input"
		<< "\n\tvalid filenames and locations when prompted." << endl;

	init_mesh_ref_frame();
	init_reference_frame();
	view = new Hullify_View("convex_hull/", &visualization_ref_frame);
	bounds = new MeshBound(mesh_ref_frame, view);
	storage = new MeshStorage();
	ocs_contact = new Ocs_listener();

	init_input_topic();
	init_mesh_name();

	//Initialize subscriber and begin waiting
	grasp_pipeline_trigger = n.subscribe("osu_grasping_trigger", 1, &MeshMaker::begin, this);
	cloud_input.resize(in_topic_name.size());
	for (unsigned int i = 0; i < in_topic_name.size(); ++i){
		cloud_input[i] = n.subscribe(in_topic_name[i], 1, &MeshMaker::convert_cloud, this);
	}

	cout << "Subscriber Created." << endl;

	visualization_msgs::Marker marker_type;
	geometry_msgs::PoseStamped pose_type;
	osu_grasp_msgs::Mesh_and_bounds openrave_type;
	geometry_msgs::PolygonStamped polygon_type;
	view->add_mesh_topic(mesh_base_name);
	view->add_topic_no_queue("principal_axis", marker_type);
	view->add_topic_no_queue("end_effector_with_offset", marker_type);
	view->add_topic_no_queue("original_third_component_axis", marker_type);
	//view->add_topic_no_queue("openrave_grasps", pose_type);
	view->add_topic_no_queue("openrave_params", openrave_type);
	//view->add_topic_no_queue("v1_in_plane", marker_type);
	//view->add_topic_no_queue("v2_in_plane", marker_type);
	view->add_topic_no_queue("ninety_degree_normal", marker_type);
	view->add_topic_no_queue("zero_degree_normal", marker_type);
	view->add_topic_no_queue("ninety_degree_plane", polygon_type);
	view->add_topic_no_queue("zero_degree_plane", polygon_type);
}

void MeshMaker::init_reference_frame()
{
	if (!ros::param::get("/convex_hull/reference_frame", visualization_ref_frame)){
		cout << "Missing /convex_hull/reference_frame parameter. Are you using a launch file?" << endl;
		exit(1);
	}
	cout << "Visualization frame: " << visualization_ref_frame << endl;
}

void MeshMaker::init_input_topic()
{
	bool using_atlas;
	if (!ros::param::get("/convex_hull/using_atlas", using_atlas)){
		cout << "Missing /convex_hull/using_atlas parameter. Are you using the launch files?" << endl;
		exit(1);
	}
		in_topic_name.reserve(3);
		if (using_atlas){
			in_topic_name.push_back("/flor/worldmodel/ocs/dist_query_pointcloud_result");
			in_topic_name.push_back("/flor/worldmodel/ocs/stereo_cloud_result");
			in_topic_name.push_back("/flor/worldmodel/ocs/cloud_result");

		} else {
			string cloud_topic;
			if (!ros::param::get("/convex_hull/cloud_input_topic", cloud_topic)){
				cout << "Could not find /convex_hull/cloud_input_topic paramter. Are you using launch files?" << endl;
				exit(1);
			}
			in_topic_name.push_back(cloud_topic);
		}

	cout << "The first input topic is " << in_topic_name[0] << endl;
}

void MeshMaker::init_mesh_ref_frame()
{
	if (!ros::param::get("convex_hull/mesh_ref_frame", mesh_ref_frame)){
		cout << "Could not find parameter convex_hull/mesh_ref_frame. Are you using the launch files?" << endl;
		exit(1);
	}
	cout << "Reference frame for the output meshes: " << mesh_ref_frame << endl;
}

void MeshMaker::init_mesh_name()
{
	mesh_base_name = "hull_mesh";
	cout << "Base mesh name: " << mesh_base_name << endl;
}

MeshMaker::~MeshMaker()
{
	delete view;
	delete bounds;
}

//Simply spin and wait for callback requests
//We are using the singly-threaded spin() function, it
//	goes to the global callback queue, finds the oldest
//	callback and executes it.
//A multithreaded option is available, but it would take up
//	system resources we just don't need. Little gain.
void MeshMaker::listen()
{
	cout << "Awaiting pointclouds..." << endl << endl;
	ros::spin();
	//ros::MultiThreadedSpinner spinner(4); //Note: queue size is 1, if you uncomment these lines, that must be changed as well.
	//spinner.spin();

}

void MeshMaker::begin(const std_msgs::String::ConstPtr& msg)
{
	cout << "Trigger received." << endl;
	if (msg->data == "L" || msg->data == "l"){
		cout << "Using Left side for grasping." << endl;
		using_left_hand = true;
	} else {
		cout << "Using right side for grasping." << endl;
		using_left_hand = false;
	}
	
	//convert_cloud();
}

/*
void MeshMaker::accept_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

}*/

void MeshMaker::set_arm_param(char arm)
{
	if (arm == 'L'){
		ros::param::set("/convex_hull/lr_grasping_arm", "L");
	} else {
		ros::param::set("/convex_hull/lr_grasping_arm", "R");
	}
}

//Function: convert_cloud
//Description: The workhorse; this function receives pointcloud
//	requests, calls qhull, updates the the mesh markers and 
//	publishes the result.
//Parameters: A pointcloud obtained from in_topic_name (set in constructor)
//Postconditions: A convex hull has been generated and published.
//	goes back to listen()
//Assumptions: This node handles one end to end computational process:
//	one callback.
void MeshMaker::convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "Pointcloud received" << std::endl;
	set_arm_param('L');
	ROS_WARN("Setting Left arm as manipulation arm permanently, for now");
	pcl::PointXYZ target_point;
	try{
		target_point = ocs_contact->get_recent_request_pt();
	} catch (int e){
		cout << "No point on object recognized, cannot segment properly." << endl;
		return;
	}
	
	//Convert PointCloud2 message to PCL's PointCloud<PointXYZ>
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (!get_cloud(msg, intermediate_cloud)){
	    	//Invalid cloud, return nothing.
	    	return;
	}

	try {
		intermediate_cloud = isolate_hull_cluster(intermediate_cloud, target_point);
	} catch (...){
		//Segmentation failure. Could be density of cloud
		return;
	}

	//Run qhull externally (or see comments just below)
	pcl::PolygonMesh::Ptr convex_hull = qhull.mk_mesh(intermediate_cloud);
	convex_hull->cloud.header = pcl_conversions::toPCL(msg->header);

	//In a system where qHull (libqhull5) is v2011.1, the below should work (untested).
	//pcl::PolygonMesh::Ptr convex_hull = mk_mesh(intermediate_cloud);
	
	//Get plane and centroid
	bounds->set_input_cloud(intermediate_cloud);
	bounds->construct_planes();
	bounds->publish_plane1();
	bounds->publish_plane2();
	bounds->publish_centroid();

	//geometry_msgs::PoseStamped wrist_pose = get_wrist_orientation(intermediate_cloud);
	//view->publish("openrave_grasps", wrist_pose);

	storage->set_mesh(convex_hull);

	string mesh_full_abs_path = view->publish_mesh(mesh_base_name, convex_hull);
	send_hull_and_planes_to_openrave(mesh_full_abs_path, convex_hull, intermediate_cloud);
}

bool MeshMaker::get_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud)
{
	cout << "MeshMaker::convert_cloud - Why does callback need const message?\n\t"
		<< "Will it change what's in the topic if we non-const it?" << endl;
	sensor_msgs::PointCloud2 temp_cloud;
	pcl_ros::transformPointCloud(mesh_ref_frame, *msg, temp_cloud, listener);
	pcl::moveFromROSMsg(temp_cloud, *intermediate_cloud);

    if (!is_valid_cloud(intermediate_cloud)){       
	return false;
    }


    return true;
}

//Determines if the cloud has enough non-coplanar points to describe
//  a convex hull.
bool MeshMaker::is_valid_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud->points.size() < 4){
		cout << "Insufficient information in pointcloud request to generate a convex hull.\n\tInput requires at least 4 points." << endl;
        return false;
	}

    long num_pts = cloud->points.size();    
    pcl::PointXYZ test_pts[4];
    test_pts[0] = cloud->points[0];
    test_pts[1] = cloud->points[1];
    geometry_msgs::Point slope;
    slope.x = test_pts[1].x - test_pts[0].x;
    slope.y = test_pts[1].y - test_pts[0].y;
    slope.z = test_pts[1].z - test_pts[0].z;
    
    long i;
    bool success = false;
    double dx, dt;    
    for (i = 2; i < num_pts; i++){
        dx = cloud->points[i].x - test_pts[1].x;
        dt = dx / slope.x;
        if ((test_pts[1].y + slope.y * dt != cloud->points[i].y) ||
            (test_pts[1].z + slope.z * dt != cloud->points[i].z)){
            success = true;
            break;
        }
    }

    if (!success){
        cout << "No non-collinear points exist in input cloud." << endl;        
        return false;
    } else {
        test_pts[2] = cloud->points[i];
    }
    
    //Find non-coplanar points:
    /*success = false;
    for (i++; i < num_pts; ++i){
        
    }*/

    return true;
}

//Believes x_axis is the palm-outward axis.
geometry_msgs::PoseStamped MeshMaker::get_wrist_orientation(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question)
{
	Axes reference_axes, goal_axes;
	reference_axes = mk_standard_coordinate_axes();
	goal_axes = get_goal_axes(pts_in_question);

	Eigen::Vector3d camera_to_centroid = bounds->get_camera_normal_vec();
	double palm_normal_camera_normal_angle = get_angle_mag_between(camera_to_centroid, goal_axes.y_axis);
	cout << "palm_normal_camera_normal_angle" << palm_normal_camera_normal_angle << endl;
	view->publish("original_third_component_axis", view->mk_vector_msg(goal_axes.y_axis));
	
	if (palm_normal_camera_normal_angle > M_PI/2){
		goal_axes.x_axis = -goal_axes.x_axis;
		goal_axes.y_axis = -goal_axes.y_axis;
	}

	Eigen::Quaterniond pose_quat = get_axes_transformation(reference_axes, goal_axes);
	Eigen::Vector3d offset = -(goal_axes.y_axis / goal_axes.y_axis.norm()) * (0.135); //Offset by 15 centimeters out of palm. (12 is connector to palm)

	view->publish("end_effector_with_offset", view->mk_pt_msg(bounds->get_centroid() + offset));
	return view->mk_pose_msg(pose_quat, bounds->get_centroid() + offset);
}

Axes MeshMaker::get_goal_axes(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question)
{
	Axes goal_axes;
	Eigen::Matrix3d principal_axes = get_principal_axes(pts_in_question);
	goal_axes.x_axis = principal_axes.col(1);
	goal_axes.y_axis = principal_axes.col(2);
	goal_axes.z_axis = principal_axes.col(0);

	if (!vecs_are_equal(goal_axes.y_axis.cross(goal_axes.z_axis), goal_axes.x_axis)){
		//cout << "We gave ourselves invalid PCA axes!!!!!" << endl;
		goal_axes.x_axis = - goal_axes.x_axis;
	}

	return goal_axes;
}

Eigen::Matrix3d MeshMaker::get_principal_axes(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question)
{
	pcl::PCA<pcl::PointXYZ> component_finder;
	component_finder.setInputCloud(pts_in_question);

	Eigen::Matrix3f principal_axes = component_finder.getEigenVectors();
	Eigen::Vector3f principal_axis = principal_axes.col(0);

	cout << "Principle axis in get_principal_axis(): " << principal_axis << endl;
	visualization_msgs::Marker pa_msg = view->mk_vector_msg(principal_axis.cast<double>());
	view->publish("principal_axis", pa_msg);

	//return principal_axis.cast<double>();
	return principal_axes.cast<double>();
}

void MeshMaker::send_hull_and_planes_to_openrave(string& mesh_full_abs_path, pcl::PolygonMesh::Ptr convex_hull, pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts)
{
	osu_grasp_msgs::Mesh_and_bounds openrave_msg;
	Eigen::Vector3d centroid = bounds->get_centroid();
	geometry_msgs::Point ros_centroid;
	ros_centroid.x = centroid[0];
	ros_centroid.y = centroid[1];
	ros_centroid.z = centroid[2];

	openrave_msg.header.frame_id = visualization_ref_frame;
	openrave_msg.header.stamp = ros::Time::now();
	openrave_msg.full_abs_mesh_path = mesh_full_abs_path;
	mk_mesh_msg(openrave_msg.convex_hull, convex_hull);
	openrave_msg.mesh_centroid = ros_centroid;
	set_bounding_planes(openrave_msg, mesh_pts);

	view->publish("openrave_params", openrave_msg);
}

void MeshMaker::set_bounding_planes(osu_grasp_msgs::Mesh_and_bounds& openrave_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts)
{
	Eigen::Vector3d p1_normal = bounds->get_plane1_normal();
	Eigen::Vector3d p2_normal = bounds->get_plane2_normal();
	Eigen::Vector3d camera_to_centroid = bounds->get_camera_normal_vec();
	Eigen::Vector3d ninety_normal; //= get_ninety_degree_base(mesh_pts);
	if (using_left_hand){
		ninety_normal = bounds->get_torso_normal_vec("l");
	} else {
		ninety_normal = bounds->get_torso_normal_vec("r");
	}

	cout << "Ninety normal: " << ninety_normal << endl;

	Eigen::Vector3d centroid = bounds->get_centroid();
	Eigen::Vector3d horiz_normal = bounds->get_horiz_normal();

	Eigen::Vector3d zero_degree_plane_normal = get_zero_degree_normal(horiz_normal, ninety_normal);

	Eigen::Vector3d ninety_degree_plane_normal = -ninety_normal;

	/*if (get_angle_mag_between(-camera_to_centroid, p1_normal) > (M_PI / 2)){
		//p2 is in the proper side
		record_planes(msg, p2_normal, p1_normal, ninety_degree_plane_normal, zero_degree_plane_normal);

	} else {
		//p1 is in the proper side
		record_planes(msg, p1_normal, p2_normal, ninety_degree_plane_normal, zero_degree_plane_normal);
	}*/

	ninety_degree_plane_normal.normalize();
	centroid -= ninety_degree_plane_normal * 0.015;

	ninety_degree_plane_normal.normalize();
	zero_degree_plane_normal.normalize();
	view->publish("ninety_degree_normal", view->mk_vector_msg(ninety_degree_plane_normal));
	view->publish("zero_degree_normal", view->mk_vector_msg(zero_degree_plane_normal));
	view->publish("ninety_degree_plane", bounds->mk_plane_msg_ext(mk_plane(centroid, ninety_degree_plane_normal), init_pt(centroid)));
	view->publish("zero_degree_plane", bounds->mk_plane_msg_ext(mk_plane(centroid, zero_degree_plane_normal), init_pt(centroid)));
	openrave_msg.ninety_degree_bounding_planes[0] = view->mk_shape_plane(*mk_plane(centroid, ninety_degree_plane_normal));
	openrave_msg.ninety_degree_bounding_planes[1] = view->mk_shape_plane(*mk_plane(centroid, zero_degree_plane_normal));
	openrave_msg.knowledge_bounding_planes[0] = view->mk_shape_plane(bounds->get_plane1());
	openrave_msg.knowledge_bounding_planes[1] = view->mk_shape_plane(bounds->get_plane2());
	openrave_msg.plane_sep_angle_gt_pi = are_planes_obtuse(p1_normal, p2_normal);
}

Eigen::Vector3d MeshMaker::get_ninety_degree_base(pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts)
{
	Eigen::Vector3d camera_to_centroid = bounds->get_camera_normal_vec();
	Eigen::Matrix3d principal_axes = get_principal_axes(mesh_pts);
	principal_axes.cast<double>();
	Eigen::Vector3d pca2 = principal_axes.col(1);
	Eigen::Vector3d pca3 = principal_axes.col(2);

	double pca2_alignment = get_angle_mag_between(camera_to_centroid, pca2);
	double pca3_alignment = get_angle_mag_between(camera_to_centroid, pca3);

	if (pca2_alignment > M_PI/2){
		pca2_alignment -= M_PI;
		pca2 *= -1;
	}

	if (pca3_alignment > M_PI/2){
		pca3_alignment -= M_PI;
		pca3 *= -1;
	}

	if (pca2_alignment < pca3_alignment){
		return pca2;
	}

	return pca3;
}

Eigen::Vector3d MeshMaker::get_zero_degree_normal(Eigen::Vector3d& horiz_normal, Eigen::Vector3d& camera_to_centroid)
{
	ROS_WARN("Be sure the object is not exactly above the robot. Code against that...");
	if (using_left_hand){
		//return horiz_normal.cross(camera_to_centroid);
		camera_to_centroid.cross(Eigen::Vector3d(0, 0, 1));
	}

	return -camera_to_centroid.cross(Eigen::Vector3d(0, 0, 1));

}


/*void MeshMaker::record_planes(hullify::Mesh_and_bounds& msg, Eigen::Vector3d& know_p_proper, Eigen::Vector3d& know_p_improper, Eigen::Vector3d& ninety_normal, Eigen::Vector3d& zero_normal)
{
	if (get_angle_mag_between(zero_normal, know_p_proper) > (M_PI / 2)){
		set_openrave_msg_planes(msg, know_p_proper, zero_normal, ninety_normal, know_p_improper);			
		openrave_msg.planes_sep_angle_gt_pi = are_planes_obtuse(ninety_normal, know_p_improper);
	} else {
		set_openrave_msg_planes(msg, ninety_normal, zero_normal, know_p_proper, know_p_improper);			
		openrave_msg.planes_sep_angle_gt_pi = are_planes_obtuse(know_p_proper, know_p_improper);
	}
}

void MeshMaker::set_openrave_msg_planes(hullify::Mesh_and_bounds& msg, Eigen::Vector3d strict_vec1, Eigen::Vector3d strict_vec2, Eigen::Vector3d relaxed_vec1, Eigen::Vector3d relaxed_vec2)
{
	Eigen::Vector3d centroid = bounds->get_centroid();
	msg.stringent_bounding_planes[0] = view->mk_shape_plane(mk_plane(centroid. strict_vec1));
	msg.stringent_bounding_planes[1] = view->mk_shape_plane(mk_plane(centroid, strict_vec2));
	msg.relaxed_bounding_planes[0] = view->mk_shape_plane(mk_plane(centroid, relaxed_vec1));
	msg.relaxed_bounding_planes[1] = view->mk_shape_plane(mk_plane(centroid, relaxed_vec2));
}*/

bool MeshMaker::are_planes_obtuse(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2)
{
	Eigen::Vector3d plane_intersection_vector = n1.cross(n2);

	Eigen::Vector3d ref_vec = n1 + n2;

	Eigen::Vector3d v1_in_plane = (n1.cross(plane_intersection_vector));
	Eigen::Vector3d v2_in_plane = -1 * n2.cross(plane_intersection_vector);

	view->publish("v1_in_plane", view->mk_vector_msg(v1_in_plane));
	view->publish("v2_in_plane", view->mk_vector_msg(v2_in_plane));

	double angle = get_angle_mag_between(ref_vec, v1_in_plane);
	cout << "Angle1: " << angle << endl;
	angle += get_angle_mag_between(ref_vec, v2_in_plane);
	cout << "Total angle: " << angle << endl;

	if (angle >= M_PI){
		cout << "Planes are obtuse!!!" << endl;
		return true;
	}

	cout << "Planes are acute!!" << endl;
	return false;
}

void MeshMaker::mk_mesh_msg(shape_msgs::Mesh& msg, pcl::PolygonMesh::Ptr convex_hull)
{
	msg.vertices.clear();
	msg.triangles.clear();

	tf::TransformListener listener;

	pcl::PointCloud<pcl::PointXYZ> cloud;//, temp_cloud;
	pcl::fromPCLPointCloud2(convex_hull->cloud, cloud);
	//tf::StampedTransform transform = get_pelvis_transform(temp_cloud.header.frame_id);
	//pcl_ros::transformPointCloud(temp_cloud, cloud, transform);
	//cout << "Conversion result: " << res << endl;

	long num_pts = cloud.points.size();
	cout << "Num pts in conversion result: " << num_pts << endl;
	geometry_msgs::Point pt;
	for (long i = 0; i < num_pts; ++i){
		pt.x = cloud.points[i].x;
		pt.y = cloud.points[i].y;
		pt.z = cloud.points[i].z;
		msg.vertices.push_back(pt);
	}
	
	long num_facets = convex_hull->polygons.size();
	shape_msgs::MeshTriangle cur_facet;
	short j;
	for (long i = 0; i < num_facets; ++i){
		for (j = 0; j < 3; ++j){
			cur_facet.vertex_indices[j] = (convex_hull->polygons)[i].vertices[j];
		}

		msg.triangles.push_back(cur_facet);
	}
}
/*
sensor_msgs::PointCloud2 MeshMaker::transform_ptcloud(const sensor_msgs::PointCloud2& in_cloud, const string& target_frame)
{
	tf::TransformListener listener(n);
	sensor_msgs::PointCloud2 out_cloud;
	listener.waitForTransform(target_frame, in_cloud.header.frame_id, ros::Time::now(), ros::Duration(2.0));
	while(1){
		try {
			listener.transformPointCloud(target_frame, in_cloud, out_cloud);
			
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();

		} catch(...){
			ros::Duration(0.1).sleep();
		}
	}

	return out_cloud;
}
*/
