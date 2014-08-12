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

#ifndef ROBOTIQ_INTERFACE_H_
#define ROBOTIQ_INTERFACE_H_

//Robotiq hand messages
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>

// ROS Control includes
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <sensor_msgs/JointState.h>

namespace RobotiqHardwareInterface
{

class RobotiqHardwareInterface
  : public hardware_interface::RobotHW
{
public:
  RobotiqHardwareInterface();

  bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const;

   // Ros Control

  void cleanup();
  void read(ros::Time time, ros::Duration period);
  void write(ros::Time time, ros::Duration period);

  bool new_data_ready_;

private:

  //Robotiq specific communication code

  void robotiq_Callback(const robotiq_s_model_control::SModel_robot_input::ConstPtr &msg);
  void InitializeRobotiq();

  hardware_interface::JointStateInterface    joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface   effort_joint_interface_;

  std::map<std::string, double> joint_position_commands_;
  std::map<std::string, double> joint_velocity_commands_;
  std::map<std::string, double> joint_effort_commands_;

  std::map<std::string, double> joint_positions_states_;
  std::map<std::string, double> joint_velocitys_states_;
  std::map<std::string, double> joint_efforts_states_;

  boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;

  ros::Publisher robotiq_output_pub_;
  ros::Subscriber robotiq_input_sub_;

  robotiq_s_model_control::SModel_robot_input   robotiq_input_msg_;
  robotiq_s_model_control::SModel_robot_output robotiq_output_msg_;

  std::string hand_side_;
  std::vector<std::string> joint_names_;
};

} //end of namespace RobotiqHardwareInterface

#endif
