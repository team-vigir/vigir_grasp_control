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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

//#include "mesh_bound.h"
#include "plane_reps_and_3dmath.h"

#include <tf/tf.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <cmath>

using std::cout;
using std::cin;
using std::endl;

#define FLOAT_TOLERANCE 0.001


struct Axes {
	Eigen::Vector3d x_axis;
	Eigen::Vector3d y_axis;
	Eigen::Vector3d z_axis;
};

Axes mk_standard_coordinate_axes();

Eigen::Quaterniond get_axes_transformation(Axes inital_axes, Axes& goal_axes);
Eigen::Quaterniond align_axes_transform(Eigen::Vector3d from_axis, Eigen::Vector3d to_axis, Eigen::Vector3d anti_parallel_rot_axis);
Eigen::Quaterniond rotate_pi_about_axis(Eigen::Vector3d axis_of_rotation);
Eigen::Quaterniond get_rotational_quaternion(Eigen::Vector3d axis_of_rotation, double angle);
double find_angular_separation(Eigen::Vector3d ref, Eigen::Vector3d line_normal, Eigen::Vector3d vec);
bool is_theta_greater_than_pi(Eigen::Vector3d line_normal, Eigen::Vector3d vec_in_question);
void perform_axial_rotation(Eigen::Quaterniond rotation, Axes& axes);
void print_quaternion(Eigen::Quaterniond quat);
void verify_transform(Eigen::Quaterniond tranform, Eigen::Vector3d initial_y_axis, Eigen::Vector3d goal_y_axis);

geometry_msgs::PoseStamped set_back_pose_on_y(geometry_msgs::PoseStamped& pose, double distance);
