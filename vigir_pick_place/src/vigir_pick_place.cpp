//=================================================================================================
// Copyright (c) 2014, Alberto Romay, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <vigir_pick_place/vigir_pick_place.h>


// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id    = "world";
//  p.pose.position.x    =  0.24277830431;
//  p.pose.position.y    = -0.745004221203;
//  p.pose.position.z    =  1.26911157683;
//  p.pose.orientation.x = -0.446351722051;
//  p.pose.orientation.y =  0.563471242384;
//  p.pose.orientation.z =  0.449446130412;
//  p.pose.orientation.w =  0.530347504081;

  p.pose.position.x    =  0.47417;
  p.pose.position.y    =  0.21743;
  p.pose.position.z    =  1.2511;
  p.pose.orientation.x = -0.68684;
  p.pose.orientation.y = -0.26322;
  p.pose.orientation.z =  0.64401;
  p.pose.orientation.w = -0.21027;

  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.y = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "l_hand";
  g.pre_grasp_approach.min_distance = 0.05;
  g.pre_grasp_approach.desired_distance = 0.1;

  g.post_grasp_retreat.direction.header.frame_id = "world";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.05;
  g.post_grasp_retreat.desired_distance = 0.1;

  g.pre_grasp_posture.joint_names.resize(5);
  g.pre_grasp_posture.joint_names[0] = "left_f0_j1";
  g.pre_grasp_posture.joint_names[1] = "left_f1_j1";
  g.pre_grasp_posture.joint_names[2] = "left_f2_j1";
  g.pre_grasp_posture.joint_names[3] = "left_f1_j0";
  g.pre_grasp_posture.joint_names[4] = "left_f2_j0";
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(5);
  g.pre_grasp_posture.points[0].positions[0] = 0.0;
  g.pre_grasp_posture.points[0].positions[1] = 0.0;
  g.pre_grasp_posture.points[0].positions[2] = 0.0;
  g.pre_grasp_posture.points[0].positions[3] = 0.0;
  g.pre_grasp_posture.points[0].positions[4] = 0.0;

  g.pre_grasp_posture.points[0].time_from_start = ros::Duration(3.0);

  g.grasp_posture.joint_names.resize(5);
  g.grasp_posture.joint_names[0] = "left_f0_j1";
  g.grasp_posture.joint_names[1] = "left_f1_j1";
  g.grasp_posture.joint_names[2] = "left_f2_j1";
  g.grasp_posture.joint_names[3] = "left_f1_j0";
  g.grasp_posture.joint_names[4] = "left_f2_j0";
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(5);
  g.grasp_posture.points[0].positions[0] = 1.0;
  g.grasp_posture.points[0].positions[1] = 1.0;
  g.grasp_posture.points[0].positions[2] = 1.0;
  g.grasp_posture.points[0].positions[3] = 0.0;
  g.grasp_posture.points[0].positions[4] = 0.0;

  g.grasp_posture.points[0].time_from_start = ros::Duration(3.0);

  g.allowed_touch_objects.resize(1);
  g.allowed_touch_objects[0] = "part";


  g.id = "KAL_grasp";

  grasps.push_back(g);
  group.setSupportSurfaceName("table");
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id    = "world";
  p.pose.position.x    =  0.5;
  p.pose.position.y    = 0.6;
  p.pose.position.z    =  1.225;
  p.pose.orientation.x =  0.0;
  p.pose.orientation.y =  0.0;
  p.pose.orientation.z =  0.0;
  p.pose.orientation.w =  1.0;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.post_place_retreat.direction.vector.y = -1.0;
  g.post_place_retreat.direction.header.frame_id = "l_hand";
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.direction.header.frame_id = "world";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;


  g.post_place_posture.joint_names.resize(5);
  g.post_place_posture.joint_names[0] = "left_f0_j1";
  g.post_place_posture.joint_names[1] = "left_f1_j1";
  g.post_place_posture.joint_names[2] = "left_f2_j1";
  g.post_place_posture.joint_names[3] = "left_f1_j0";
  g.post_place_posture.joint_names[4] = "left_f2_j0";
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(5);
  g.post_place_posture.points[0].positions[0] = 0;
  g.post_place_posture.points[0].positions[1] = 0;
  g.post_place_posture.points[0].positions[2] = 0;
  g.post_place_posture.points[0].positions[3] = 0;
  g.post_place_posture.points[0].positions[4] = 0;

  g.post_place_posture.points[0].time_from_start = ros::Duration(3.0);

  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "l_hand";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  //  group.setPathConstraints(constr);
  //group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "vigir_pick_controller");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("l_arm_group");
  group.setPlanningTime(15.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "world";

  // remove pole
  co.id = "obstacle";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.01;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.67;
  co.primitive_poses[0].position.y = 0.67;
  co.primitive_poses[0].position.z = 1.575;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);



  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.2;
  co.primitive_poses[0].position.z = 0.9;
  pub_co.publish(co);



  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  pub_aco.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = 0.69;
  co.primitive_poses[0].position.y = -0.06;
  co.primitive_poses[0].position.z = 1.225;

  tf::Quaternion part_quat;

  part_quat.setEuler(0.0,0.0,-0.92);

  co.primitive_poses[0].orientation.x = part_quat.x();
  co.primitive_poses[0].orientation.y = part_quat.y();
  co.primitive_poses[0].orientation.z = part_quat.z();
  co.primitive_poses[0].orientation.w = part_quat.w();
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();
  return 0;
}
