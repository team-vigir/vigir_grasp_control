#include "ros_interface.h"


void check_client_validity(ros::ServiceClient& client)
{
	if (client.isValid()){
		cout << "We're getting that the client is valid..." << endl;
	} else {
		cout << "We're getting an invalid client..." << endl;
	}

}

void stall_after_grasp_dist_check(bool success)
{
	if (success){
		cout << "Grasp is within acceptable distance." << endl;
	} else {
		cout << "Grasp rejected!" << endl;
	}

	string input;
	cin >> input;

}

string halt_for_input(string display_string)
{
	cout << display_string << endl;
	string input;
	std::getline(cin, input);

	return input;
}


Ros_Int::Ros_Int() {
	cout << "Wrong constructor called!" << endl;
	exit(1);
}

Ros_Int::Ros_Int(RobotBasePtr robot, Jc_Mods* changes)
{
	int argc = 0;
	char* argv[1];
	argv[0] = NULL;
	//ros::init(argc, argv, "openrave_grasper_plugin");
	ros::init(argc, argv, "openrave_grasper_plugin", ros::init_options::AnonymousName );
	n = new ros::NodeHandle;
	//cout << "Made a node handle!";
	this->changes = changes;

	client = n->serviceClient<osu_grasp_msgs::CheckGraspDistance>("CheckGraspDistance");
	moveit_ik_client = n->serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
	moveit_ik_solution_pub = n->advertise<moveit_msgs::RobotState>("convex_hull/moveit_ik_results", 5);
	moveit_ik_solution_pose_pub = n->advertise<geometry_msgs::PoseStamped>("convex_hull/moveit_ik_reachable_pose", 5);
	moveit_service_request_fully_initalized = false;
	
	this->robot = robot;
	this->l_manip = robot->GetActiveManipulator();
	this->l_hand = l_manip->GetEndEffector();

	//start_robot_state_subscriber();
	init_moveit_ik_service_template();
	if (!ros::param::get("/convex_hull/mesh_ref_frame", pose_frame)){
		ROS_ERROR("Could not get parameter convex_hull/mesh_ref_frame, are you using launch files?");
		exit(1);
	}

	pregrasp_offset = 0;
	
	ros::AsyncSpinner spinny(2);
	spinny.start();
}

void Ros_Int::init_moveit_ik_service_template()
{
	template_service_request.request.ik_request.avoid_collisions = false;
	cout << "Setting MoveIt IK call to disregard collisions, just to see if it works." << endl;

	template_service_request.request.ik_request.timeout = ros::Duration(0.1);
	template_service_request.request.ik_request.attempts = 1;

	//We can leave the robot state empty for now. Using seeds will reduce time requirements
	//	But, the robot state is proving hard to get...

	moveit_service_request_fully_initalized = true;
}
/*
void Ros_Int::init_moveit_ik_constraints_template(RobotBase::ManipulatorPtr manip)
{

	for (int i = 0; i < arm_joints.size(); ++i){
		temp_joint_const.joint_name = robot->GetJointFromDOFIndex(arm_joints[i]).GetName();
		temp_

		template_joint_limit_constraints.joint_constraints.push_back(
	}
}
*/

void Ros_Int::start_robot_state_subscriber()
{
	ROS_DEBUG("Subscribing to robot state display from moveit");
	bool wait_for_moveit = ros::service::waitForService("compute_ik");
	if (!wait_for_moveit){
		ROS_ERROR("Could not connect to moveit to get a template robot state");
		exit(1);
	}

	robot_state_sub = n->subscribe("joint_states", 1, &Ros_Int::robot_state_callback, this);
}

void Ros_Int::robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	robot_state_sub.shutdown();
	
	cout << "Got robot state message!" << endl;
	
	template_robot_state.joint_state = *msg;
	template_robot_state.is_diff = false;

	template_service_request.request.ik_request.robot_state = template_robot_state;

	ROS_ERROR("Built template state. Did not consider multiDOF joints or attached objects.");

	seed_pre_reachability_pose();
	moveit_service_request_fully_initalized = true;

}

/*void Ros_Int::dist_resp_callback(const osu_grasp_msgs::CheckGraspDistanceResponse::ConstPtr& msg){
	cout << "Callback online!" << endl;
	got_result = true;
	cur_grasp_within_dist = msg->within_acceptable_dist;
}
*/

bool Ros_Int::check_hand_position(TrajectoryBasePtr traj) 
{
	ros::param::get("/convex_hull/pregrasp_offset", pregrasp_offset);
	//ROS_INFO_STREAM("Pregrasp offset: " << pregrasp_offset);

	std::vector<dReal> data;
	traj->GetWaypoint(-1, data, robot->GetConfigurationSpecification());

	robot->SetConfigurationValues(data.begin(), true);

	Transform hand_pose = l_hand->GetTransform();
	Transform robot_pose = robot->GetTransform();
	if (!is_null_transform(robot_pose)){
		//ROS_ERROR("Found robot does not sit at world center!");
		//cout << "robot transform: " << robot_pose << endl;
		//hand_pose = robot_pose.inverse() * hand_pose; //Doesn't work... returns the same pose every time...
	}

	geometry_msgs::PoseStamped ros_hand_pose = get_hand_pose(hand_pose);
	double center_to_palm_face_dist = get_hand_width(l_hand);
	double acceptable_dist = center_to_palm_face_dist + 0.05;
	

	osu_grasp_msgs::CheckGraspDistanceRequest srv;
	osu_grasp_msgs::CheckGraspDistanceResponse resp;
	srv.hand_pose = ros_hand_pose;
	srv.acceptable_dist = acceptable_dist;

	check_client_validity(client);
	//cout << "Just before the call()" << endl;
	if (client.call(srv, resp)){
		//stall_after_grasp_dist_check(resp.within_acceptable_dist);
		if (!resp.within_acceptable_dist){
			ret_msg = "\t\x1b[34m Palm too far from object \x1b[37m\n";
			cout << ret_msg << endl;
			changes->write_log(ret_msg);
			return false;
		} else {
			//cout << "\tPalm NOT too far from object. Service call returned success." << endl;
		}
	} else {
		ROS_ERROR("Distance check call error.");
	}

	if (!pose_is_reachable(ros_hand_pose) || !pregrasp_pose_is_reachable(ros_hand_pose)){
		ret_msg = "\t\x1b[35m IK Reachability Failed \x1b[37m\n";
		cout << ret_msg << endl;
		changes->write_log(ret_msg);

		return false;
	} else {
		//cout << "\tReachability success." << endl;
	}

	return true;
}

//Check for identity matrix
bool Ros_Int::is_null_transform(Transform transform)
{
	if (transform.trans.x == 0 && transform.trans.y == 0 && transform.trans.z == 0 && (transform.rot.x == 1))
		return true;
	return false;
}

geometry_msgs::PoseStamped Ros_Int::get_hand_pose(Transform& pose)
{
	RaveVector<double> v_pos = pose.trans;
	RaveVector<double> v_rot = pose.rot;
	geometry_msgs::PoseStamped out_pose;
	out_pose.pose.position.x = v_pos[0];
	out_pose.pose.position.y = v_pos[1];
	out_pose.pose.position.z = v_pos[2];

	out_pose.pose.orientation.x = v_rot.y;
	out_pose.pose.orientation.y = v_rot.z;
	out_pose.pose.orientation.z = v_rot.w;
	out_pose.pose.orientation.w = v_rot.x;

	out_pose.header.frame_id = pose_frame;
	out_pose.header.stamp = ros::Time::now();
	return out_pose;
}

double Ros_Int::get_hand_width(KinBody::LinkPtr hand)
{
	//ROS_ERROR("Hardcoded hand width, we need to change that!");
	//RaveVector<double> hand_box = hand->GetGeometry(0)->GetBoxExtents();
	//cout << "hand_box: " << hand_box << endl;
	return 0.06; //see pg 91 of robotiq manual
}

geometry_msgs::Point Ros_Int::get_approach_dir(RobotBase::ManipulatorPtr manip)
{
	Vector dir = manip->GetLocalToolDirection();
	Transform ee_pose = manip->GetTransform();

	Vector global_dir = ee_pose.inverse().rotate(dir);
	geometry_msgs::Point out_dir;
	out_dir.x = global_dir[0];
	out_dir.y = global_dir[1];
	out_dir.z = global_dir[2];

	return out_dir;
}

bool Ros_Int::pose_is_reachable(geometry_msgs::PoseStamped hand_pose)
{
	//halt_for_input("Starting pose_is_reachable()");

	if (!moveit_service_request_fully_initalized){
		ROS_ERROR("Moveit IK service request template in ros_interface.cpp is still false. No robot display message received.");
		return false;
	}

	get_ik_group();
	add_joint_constraints();
	template_service_request.request.ik_request.pose_stamped = hand_pose;

	return mk_ik_call();
}

bool Ros_Int::pregrasp_pose_is_reachable(geometry_msgs::PoseStamped hand_pose)
{
	geometry_msgs::PoseStamped offset_pose = set_back_pose_on_y(hand_pose, pregrasp_offset);
	template_service_request.request.ik_request.pose_stamped = offset_pose;
	template_service_request.request.ik_request.robot_state = template_service_request.response.solution;

	add_joint_constraints();

	return mk_ik_call();
}

bool Ros_Int::mk_ik_call()
{
	if (moveit_ik_client.call(template_service_request)){
		if (template_service_request.response.error_code.val == 1){
			//halt_for_input("Grasp position reachable!");
			//cout << template_service_request.response.solution.joint_state << endl;
			moveit_ik_solution_pub.publish(template_service_request.response.solution);
			moveit_ik_solution_pose_pub.publish(template_service_request.request.ik_request.pose_stamped);
			//halt_for_input("Does that pose work?");
			return true;

		} else {
			return analyze_moveit_error_code(template_service_request.response.error_code.val);


		}
	} else {
		ROS_ERROR("Openrave IK service call failed");
	}

	return false;
}

//For testing, we will set the pose to something that we know is reachable,
//	We will essentially sidestep the generation to see if it works at all.
void set_to_reachable_pose(geometry_msgs::PoseStamped& hand_pose)
{
	cout << "\tUsing reachable pose (starting pose of robot)" << endl;
	hand_pose.header.frame_id = "pelvis";
	hand_pose.pose.position.x = 0.0583645;
	hand_pose.pose.position.y = 0.42214;
	hand_pose.pose.position.z = -0.234274;
	hand_pose.pose.orientation.w = 0.752704;
	hand_pose.pose.orientation.x = -0.635183;
	hand_pose.pose.orientation.y = -0.007197;
	hand_pose.pose.orientation.z = -0.172994;
}

void Ros_Int::get_ik_group()
{
	if (!ros::param::get("convex_hull/using_atlas", using_atlas)){
		ROS_ERROR("Missing convex_hull/using_atlas parameter. Are you using the launch files?");
		exit(1);
	}

	if (using_atlas){
		get_ik_group_atlas();
	} else {
		template_service_request.request.ik_request.group_name = l_manip->GetName();
		template_service_request.request.ik_request.ik_link_names.clear();
		template_service_request.request.ik_request.ik_link_names.push_back(l_hand->GetName());
	}
	
} 

void Ros_Int::get_ik_group_atlas()
{
	//string arm;
	string arm_param_name = "/convex_hull/lr_grasping_arm";
	bool lr_arm_param_exists = ros::param::get(arm_param_name, arm_for_grasping);
	if (!lr_arm_param_exists){
		ROS_ERROR_STREAM("Could not find the arm being used in the parameter " << arm_param_name);
		exit(1);
	}

	if (arm_for_grasping == "L"){
		template_service_request.request.ik_request.group_name = "l_arm_group";
		//template_service_request.request.ik_request.group_name = "l_arm_with_torso_group";
		//template_service_request.request.ik_request.ik_link_name = "left_palm";
		template_service_request.request.ik_request.ik_link_names.clear();
		template_service_request.request.ik_request.ik_link_names.push_back("left_palm");

	} else if (arm_for_grasping == "R") {
		template_service_request.request.ik_request.group_name = "r_arm_group";
		template_service_request.request.ik_request.ik_link_name = "right_palm";
	} else {
		ROS_ERROR_STREAM("Found unsupported arm choice in " << arm_param_name << " param: " << arm_for_grasping);
		exit(1);
	}

	//halt_for_input(template_service_request.request.ik_request.group_name);
}

bool Ros_Int::add_joint_constraints()
{
	double tolerance_percent;
	if (!ros::param::get("/convex_hull/ik_joint_limit_tolerance", tolerance_percent)){
		ROS_ERROR("Could not get parameter /convex_hull/ik_joint_limit_tolerance from parameter server.");
	}

	if (tolerance_percent == 0){
		return true;
	} else {
		vector<int> joint_indices;
		if (!using_atlas || arm_for_grasping == "L"){
			joint_indices = l_manip->GetArmIndices();

		} else if (arm_for_grasping == "R"){
			ROS_INFO("This joint limit tolerances checks have not been written yet for the right arm. Look at Ros_Int::ik_is_within_joint_tolerances().");
			return true;

		} else {
			ROS_ERROR_STREAM("Found unsupported arm choice " << arm_for_grasping << " in Ros_Int::ik_is_within_joint_tolerances().");
			halt_for_input("Terminating, press anything to continue");
			exit(1);
			
		}

		return set_moveit_ik_call_tolerance(joint_indices, tolerance_percent);
	}
}

bool Ros_Int::set_moveit_ik_call_tolerance(vector<int>& joint_indices, double tolerance_percent)
{
	string joint_name;
	moveit_msgs::JointConstraint temp_joint_const;
	temp_joint_const.position = 0;
	temp_joint_const.weight = 1;
	
	int num_robot_joints = template_service_request.response.solution.joint_state.name.size();
	KinBody::JointPtr cur_joint;
	vector< dReal > lower_limits;
	vector< dReal > upper_limits;
	for (int i = 0; i < joint_indices.size(); ++i){
		cur_joint = robot->GetJointFromDOFIndex(joint_indices[i]);
		
		joint_name = cur_joint->GetName();
		temp_joint_const.joint_name = joint_name;
		cur_joint->GetLimits(lower_limits, upper_limits);

		/*bool found_joint = false;
		for (int j = 0; j < num_robot_joints; ++j){
			if (joint_name == template_service_request.response.solution.joint_state.name[j]){
				found_joint = true;
				ik_joint_position = template_service_request.response.solution.joint_state.position[j];
				ROS_INFO_STREAM("Joint: " << joint_name << " has moveit position " << ik_joint_position 
							<< " and openrave has limits " << lower_limits[0] << " and  " << upper_limits[0]);
				//halt_for_input("Please press enter:");
				if (	!ik_joint_position > (1 - tolerance_percent) * lower_limits[0] ||
					!ik_joint_position < (1 - tolerance_percent) * upper_limits[0])
					//Joint out of tolerance limits
					return false;
			} else {
				ROS_INFO_STREAM("Moveit joint did not match given joint: " << template_service_request.response.solution.joint_state.name[j]);
			}
		}

		if (!found_joint){
			ROS_ERROR_STREAM("In checking joint limits, could not find joint '" << joint_name << "'");
		}*/

		//OpenRAVE wont say that it is a static joint, it just zeroes the limits
		if (upper_limits[0] == 0 && lower_limits[0] == 0){
			continue;	//Since it's fixed
		}

		temp_joint_const.tolerance_above = upper_limits[0] * (1 - tolerance_percent);
		temp_joint_const.tolerance_below = lower_limits[0] * (1 - tolerance_percent) * -1;
		template_joint_limit_constraints.joint_constraints.push_back(temp_joint_const);
		//ROS_INFO_STREAM("Adding constraint for joint " << joint_name << ". Upper: " << temp_joint_const.tolerance_above << " Lower: " << temp_joint_const.tolerance_below);
	}
	
	template_service_request.request.ik_request.constraints = template_joint_limit_constraints;
	//halt_for_input("How do those joint limits look?");
	return true;
}

bool Ros_Int::analyze_moveit_error_code(int error_code)
{
	cout << "Moveit IK service error: " << endl;

	switch (error_code) {
	case 99999:	//Plain Failure
	case -31:	//IK failures
		cout << "\tSimply no solution" << endl;
		return false;
	case -10:
		ROS_WARN("\tStart state in collision. Need another start state. This is handleable...");
		break;
	case -17:
		ROS_ERROR("Invalid robot state. We shouldn't get this error if we get initial joint state from OCS");
		break;
	case -15:
		ROS_ERROR_STREAM("Apparently the group name" << template_service_request.request.ik_request.group_name  << "is invalid. That is fixable... Is it because the service call reset it?");
		break;
	default:
		ROS_WARN_STREAM("This IK error code is unexpected: code-" << error_code);

	}

	halt_for_input("Error code above?");

	return false;
}



//This function will set the position of the robot prior to calling IK
//	to a place in the middle of its grasping region. This should seed the pre-IK pose
//	and speed up IK calls.
void Ros_Int::seed_pre_reachability_pose()
{
	cout << "Write seed_pre_reachability_pose() to use a good position found by moving the arm in the OCS with joint control" << endl;
}

void Jc_Mods::init_timer()
{
	if (gettimeofday(&now, NULL)){
		ROS_ERROR("Trouble with getting the current time! Terminating...");
		exit(1);
	} else {
		current_msg = "Timer started: ";
		write_msg();
		print_wall_clock_time();
	}
}

void Jc_Mods::print_wall_clock_time()
{
	ss << now.tv_sec << "s " << now.tv_usec << " us ";
	current_msg = ss.str();
	write_msg();
}

void Jc_Mods::end_timing_session(string description_of_completed_task)
{
	struct timeval later;
	if (gettimeofday(&later, NULL)){
		cout << "Trouble in end_timing_session." << endl;
		return;
	}

	ss << "Completing " << description_of_completed_task << " took " << later.tv_sec - now.tv_sec << " s and " << later.tv_usec - now.tv_usec << " us to complete.";

	current_msg = ss.str();
	write_msg();

	current_msg = "\tequivalent process timing: " + end_ptime_session();
	write_msg();

	now = later;
}

void Jc_Mods::init_ptimer()
{
	ptime = clock();
	if (ptime != -1){
		ss << "Started process timer: " << ptime / (double) CLOCKS_PER_SEC;
		current_msg = ss.str();
	} else {
		current_msg = "Process timer online";
	}

	write_msg();
}

string Jc_Mods::end_ptime_session()
{
	time_t later = clock();
	ss << (later - ptime) / (double) CLOCKS_PER_SEC << " s process time";
	string res = ss.str();
	ss.str("");
	ptime = later;
	return res;
}

//Description: prints the message to the screen with a newline and
//	CLEARS THE STRING STREAM
void Jc_Mods::write_msg()
{
	//cout << current_msg << endl;
	current_msg += "\n";
	//RAVELOG_VERBOSE(current_msg);
	log_file.write(current_msg.c_str(), current_msg.length());
	ss.str("");
}

void Jc_Mods::write_log(string msg)
{
	log_file.write(msg.c_str(), msg.length());
}
