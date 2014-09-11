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
  p.header.frame_id    = "/world";
  p.pose.position.x    =  0.239987597961;
  p.pose.position.y    = -0.708906810204;
  p.pose.position.z    =  1.26838464158;
  p.pose.orientation.x = -0.454574987236;
  p.pose.orientation.y =  0.677787964489;
  p.pose.orientation.z =  0.288904518079;
  p.pose.orientation.w =  0.500498986619;

  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.y = -1.0;
  g.pre_grasp_approach.direction.header.frame_id = "r_hand";
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = "world";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  g.pre_grasp_posture.joint_names.resize(3);
  g.pre_grasp_posture.joint_names[0] = "right_f0_j1";
  g.pre_grasp_posture.joint_names[1] = "right_f1_j1";
  g.pre_grasp_posture.joint_names[2] = "right_f2_j1";
  g.pre_grasp_posture.points.resize(3);
  g.pre_grasp_posture.points[0].positions.resize(3);
  g.pre_grasp_posture.points[0].positions[0] = 0;
  g.pre_grasp_posture.points[0].positions[1] = 0;
  g.pre_grasp_posture.points[0].positions[2] = 0;
  g.pre_grasp_posture.points[1].positions.resize(3);
  g.pre_grasp_posture.points[1].positions[0] = 0;
  g.pre_grasp_posture.points[1].positions[1] = 0;
  g.pre_grasp_posture.points[1].positions[2] = 0;
  g.pre_grasp_posture.points[2].positions.resize(3);
  g.pre_grasp_posture.points[2].positions[0] = 0;
  g.pre_grasp_posture.points[2].positions[1] = 0;
  g.pre_grasp_posture.points[2].positions[2] = 0;

  g.grasp_posture.joint_names.resize(3);
  g.grasp_posture.joint_names[0] = "right_f0_j1";
  g.grasp_posture.joint_names[1] = "right_f1_j1";
  g.grasp_posture.joint_names[2] = "right_f2_j1";
  g.grasp_posture.points.resize(3);
  g.grasp_posture.points[0].positions.resize(3);
  g.grasp_posture.points[0].positions[0] = 1;
  g.grasp_posture.points[0].positions[1] = 1;
  g.grasp_posture.points[0].positions[2] = 1;
  g.grasp_posture.points[1].positions.resize(3);
  g.grasp_posture.points[1].positions[0] = 1;
  g.grasp_posture.points[1].positions[1] = 1;
  g.grasp_posture.points[1].positions[2] = 1;
  g.grasp_posture.points[2].positions.resize(3);
  g.grasp_posture.points[2].positions[0] = 1;
  g.grasp_posture.points[2].positions[1] = 1;
  g.grasp_posture.points[2].positions[2] = 1;

  g.id = "KAL_grasp";

  grasps.push_back(g);
  group.setSupportSurfaceName("table");
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "pelvis";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "pelvis";
  g.pre_place_approach.direction.header.frame_id = "r_hand";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  g.post_place_posture.joint_names.resize(1, "r_hand");
  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(1);
  g.post_place_posture.points[0].positions[0] = 1;

  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_hand";
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
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("r_arm_group");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "pelvis";

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
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.675;
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
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.0;
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

  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.z = 0.325;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();
  return 0;
}
