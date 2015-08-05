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
#include "pose_transform.h"

Axes mk_standard_coordinate_axes()
{
	Axes reference_axes;
	reference_axes.x_axis = Eigen::Vector3d(1, 0, 0);
	reference_axes.y_axis = Eigen::Vector3d(0, 1, 0);
	reference_axes.z_axis = Eigen::Vector3d(0, 0, 1);

	return reference_axes;
}

void print_quaternion(Eigen::Quaterniond quat)
{
	cout << "x: " << quat.x() << " y: " << quat.y() << " z: " << quat.z() << " w: " << quat.w() << endl;
}

//Preconditions: Valid, orthogonal axes are given.
//	such axes do not have a translation component, they are centered.
Eigen::Quaterniond get_axes_transformation(Axes initial_axes, Axes& goal_axes)
{
	Eigen::Quaterniond transform(1, 0, 0, 0);

	transform = align_axes_transform(initial_axes.z_axis, goal_axes.z_axis, initial_axes.x_axis) * transform;
	perform_axial_rotation(transform, initial_axes);
	
	Eigen::Vector3d negative_x_axis = -1 * initial_axes.x_axis;
	if (vecs_are_equal(initial_axes.x_axis, goal_axes.x_axis)){
		cout << "No x-axis rotation required." << endl;

	} else if (vecs_are_equal(negative_x_axis, initial_axes.x_axis)){
		transform = rotate_pi_about_axis(initial_axes.z_axis) * transform;
		cout << "180 degree rotation about z-axis to align x axes." << endl;

	} else {
		double rotation_angle = find_angular_separation(initial_axes.x_axis, initial_axes.y_axis, goal_axes.x_axis);
		cout << "Rotation angle for x axes alignment: " << rotation_angle << endl;
		transform = get_rotational_quaternion(initial_axes.z_axis, rotation_angle) * transform;
		cout << "Rotational quaternion: " << endl; 
		print_quaternion(get_rotational_quaternion(initial_axes.z_axis, rotation_angle));
	}
	
	//cout << "Use the Y axis to verify the transformation in pose_transform.cpp" << endl;
	verify_transform(transform, initial_axes.y_axis, goal_axes.y_axis);

	return transform;
}

Eigen::Quaterniond align_axes_transform(Eigen::Vector3d from_axis, Eigen::Vector3d to_axis, Eigen::Vector3d anti_parallel_rot_axis)
{
	Eigen::Quaterniond transform(1, 0, 0, 0), temp;
	Eigen::Vector3d negative_axis = -1 * to_axis;
	if (vecs_are_equal(from_axis, to_axis)){
		//No change;
		cout << "No change in z-axis direction required." << endl;
		cout << "initial: " << from_axis << endl << "goal: " << to_axis << endl;

	} else if(vecs_are_equal(from_axis, negative_axis)) {
		transform = rotate_pi_about_axis(anti_parallel_rot_axis);
		cout << "Rotation 180 degrees about the x-axis is required for the z-axis" << endl;
		
	} else {
		transform.setFromTwoVectors(from_axis, to_axis);
		transform.normalize();
		cout << "Abritrary transform required to align the z axes" << endl;
		
	}

	cout << "To align z-axis: "; print_quaternion(transform); cout << endl;
	return transform;	
}

Eigen::Quaterniond rotate_pi_about_axis(Eigen::Vector3d axis_of_rotation)
{
	double quat_w = 0;
	double quat_x = axis_of_rotation[0];
	double quat_y = axis_of_rotation[1];
	double quat_z = axis_of_rotation[2];

	Eigen::Quaterniond orientation_change(quat_w, quat_x, quat_y, quat_z);
	orientation_change.normalize();

	return orientation_change;
}

Eigen::Quaterniond get_rotational_quaternion(Eigen::Vector3d axis_of_rotation, double angle)
{
	double half_angle = angle / 2;
	double quat_w = cosf(half_angle);
	double quat_x = axis_of_rotation[0] * sinf(half_angle);
	double quat_y = axis_of_rotation[1] * sinf(half_angle);
	double quat_z = axis_of_rotation[2] * sinf(half_angle);

	Eigen::Quaterniond rotation_quat(quat_w, quat_x, quat_y, quat_z);
	rotation_quat.normalize();

	return rotation_quat;
}

void perform_axial_rotation(Eigen::Quaterniond rotation, Axes& axes)
{
	rotation.normalize();
	Eigen::Matrix<double,3,3> rotation_matrix = rotation.toRotationMatrix();
	axes.x_axis = rotation_matrix * axes.x_axis;
	axes.y_axis = rotation_matrix * axes.y_axis;
	axes.z_axis = rotation_matrix * axes.z_axis;
}

double find_angular_separation(Eigen::Vector3d ref, Eigen::Vector3d line_normal, Eigen::Vector3d vec)
{
	bool theta_greater_than_pi;
	double rotation_angle = get_angle_mag_between(vec, ref);

	cout << "rotation angle preadjustment: " << rotation_angle << endl;

	theta_greater_than_pi = is_theta_greater_than_pi(line_normal, vec);
	if (theta_greater_than_pi){
		rotation_angle = (2 * M_PI) - rotation_angle;

	}

	cout << "Rotation angle in find_angular_separation(): " << rotation_angle << endl;
	return rotation_angle;
}

bool is_theta_greater_than_pi(Eigen::Vector3d line_normal, Eigen::Vector3d vec_in_question)
{
	Eigen::Vector3d p(vec_in_question[0], vec_in_question[1], 0);
	Eigen::Vector3d n(line_normal[0], line_normal[1], 0);
	double theta = get_angle_mag_between(p, n);
	
	if (theta > (M_PI / 4)){
		cout << "Vector is opposite of normal." << endl;
		return true;
	}

	cout << "Vector is on same side as normal." << endl;
	return false;
}

void verify_transform(Eigen::Quaterniond transform, Eigen::Vector3d initial_y_axis, Eigen::Vector3d goal_y_axis)
{
	Eigen::Vector3d rotated_y_axis = transform._transformVector(initial_y_axis);
	if (vecs_are_equal(rotated_y_axis, goal_y_axis, 0.05)){
		cout << "Y-axis verifies transform correctness." << endl;
	} else {
		cout << "Invalid transform in pose_transform: y-axes do not align at end. Are supplied goal and final axes valid axes?" << endl;
		cout << "Goal y-axis: " << goal_y_axis << endl << "transformed y axis: " << endl << rotated_y_axis <<  endl;
	}
}


geometry_msgs::PoseStamped set_back_pose_on_y(geometry_msgs::PoseStamped& pose, double distance)
{
	Eigen::Quaterniond wrist_transform(pose.pose.orientation.w,
					pose.pose.orientation.x,
					pose.pose.orientation.y,
					pose.pose.orientation.z);
	Eigen::Vector3d palm_normal(0, 1, 0);
	palm_normal = wrist_transform._transformVector(palm_normal);
	palm_normal.normalize();

	if (distance > 0){
		distance = -distance;
	}

	Eigen::Vector3d offset = palm_normal * distance;
	
	geometry_msgs::PoseStamped offset_pose = pose;
	offset_pose.pose.position.x += offset[0];
	offset_pose.pose.position.y += offset[1];
	offset_pose.pose.position.z += offset[2];
	
	return offset_pose;
}
