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
#ifndef JC_MODS_H
#define JC_MODS_H
#define _C_POSIX_SOURCE 200809L

//#define GRASPCOMMAND_TIMING 1
#define CONVEX_HULL_TIMING 1
#define RECORD_CONTACTS 1

#include "plugindefs.h"
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "osu_grasp_msgs/CheckGraspDistance.h"
#include <pose_transform.h>

#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/Constraints.h"

#include <openrave/kinbody.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <sys/time.h>
#include <time.h>

using std::string;
using std::stringstream;
using std::cout;
using std::ofstream;
using namespace OpenRAVE;

string halt_for_input(string display_string);
void check_client_validity(ros::ServiceClient& client);
void stall_after_grasp_dist_check(bool success);
void set_to_reachable_pose(geometry_msgs::PoseStamped& hand_pose);

class Jc_Mods;

class Ros_Int{
public:
	Ros_Int();
	Ros_Int(RobotBasePtr robot, Jc_Mods* changes);

	bool check_hand_position(TrajectoryBasePtr traj);
	bool is_null_transform(Transform transform);
	geometry_msgs::PoseStamped get_hand_pose(Transform& pose);
	double get_hand_width(KinBody::LinkPtr hand);
	geometry_msgs::Point get_approach_dir(RobotBase::ManipulatorPtr manip);
//	void dist_resp_callback(const osu_grasp_msgs::CheckGraspDistanceResponse::ConstPtr& msg);
	void robot_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

private:
	void start_robot_state_subscriber();
	void init_moveit_ik_service_template();
//	void init_moveit_ik_constraints_template(RobotBase::ManipulatorPtr manip);
	void seed_pre_reachability_pose();
	bool pose_is_reachable(geometry_msgs::PoseStamped hand_pose);
	bool pregrasp_pose_is_reachable(geometry_msgs::PoseStamped hand_pose);
	void get_ik_group();
	void get_ik_group_atlas();
	bool mk_ik_call();
	bool add_joint_constraints();
	bool set_moveit_ik_call_tolerance(vector<int>& joint_indices, double tolerance_percent);
	bool analyze_moveit_error_code(int error_code);

	ros::NodeHandle* n;
	ros::Publisher moveit_ik_solution_pub;
	ros::Publisher moveit_ik_solution_pose_pub;
	ros::ServiceClient client;
	bool using_atlas;

	RobotBasePtr robot;
	RobotBase::ManipulatorPtr l_manip;
	KinBody::LinkPtr l_hand;
	string pose_frame;
	string arm_for_grasping;

	ros::ServiceClient moveit_ik_client;
	moveit_msgs::GetPositionIK template_service_request;
	moveit_msgs::RobotState template_robot_state;
	moveit_msgs::Constraints template_joint_limit_constraints;
	ros::Subscriber robot_state_sub;
	bool moveit_service_request_fully_initalized;
	double pregrasp_offset;

	Jc_Mods* changes;
	string ret_msg;
};


class Jc_Mods{
public:
	Jc_Mods() {cout << "Need robot ptr!" << endl; exit(1); };
	Jc_Mods(RobotBasePtr robot) : interface(robot, this) {
		ss << ros::package::getPath("rave_to_moveit") << "/logs/ravelog";
		ss << time(NULL);
		string log_path = ss.str();
		log_file.open(log_path.c_str());

		if (!log_file.good()){
			ROS_ERROR_STREAM("Could not open log file " << log_path << "!");
			exit(1);
		} /*else {
			halt_for_input(log_path);
		}*/
	};
	~Jc_Mods() { log_file.close(); };

	Ros_Int interface;

	void init_timer();
	void end_timing_session(string description_of_completed_task);
	void print_wall_clock_time();

	void init_ptimer();
	string end_ptime_session();

	void write_log(string msg);
private:
	void write_msg();

	struct timeval now;
	time_t ptime;
	stringstream ss;
	string current_msg;
	ofstream log_file;
};
#endif
