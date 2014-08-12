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

#include <controller_manager/controller_manager.h>
#include <flor_grasp_controllers/vigir_robotiq_ros_controller.h>

#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPR_TO_BYTE    496.785549105
#define PER_TO_BYTE      2.55

namespace RobotiqHardwareInterface
{

void RobotiqHardwareInterface::robotiq_Callback(const robotiq_s_model_control::SModel_robot_input::ConstPtr &msg)
{
    robotiq_input_msg_ = *msg;
    new_data_ready_    = true;
}

RobotiqHardwareInterface::RobotiqHardwareInterface()
{
    ros::NodeHandle private_nh("~");
    ros::NodeHandle* rosnode = new ros::NodeHandle();

    rosnode->setCallbackQueue(&subscriber_queue_);

    private_nh.param<std::string>("hand_side", hand_side_, std::string("left"));

    std::string param = "joint_names";

    XmlRpc::XmlRpcValue joint_names_xmlrpc;
    if (private_nh.hasParam(param))
    {
        private_nh.getParam(param, joint_names_xmlrpc);
    }
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
        throw std::invalid_argument(param);
    }

    for (int i=0; i<joint_names_xmlrpc.size(); i++)
    {
        joint_names_.push_back(hand_side_ + "_" + static_cast<std::string>(joint_names_xmlrpc[i]));
        ROS_INFO("Joint name %s added to %s Robotiq",joint_names_[i].c_str() ,hand_side_.c_str());
    }

    ROS_INFO("%s Robotiq joint names size: %d",hand_side_.c_str(),(int)joint_names_.size());

    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
        joint_velocity_commands_[joint_names_[i]] = 0.0;
        joint_position_commands_[joint_names_[i]] = 0.0;
        joint_effort_commands_[  joint_names_[i]] = 0.0;

        joint_positions_states_[ joint_names_[i]] = 0.0;
        joint_velocitys_states_[ joint_names_[i]] = 0.0;
        joint_efforts_states_[   joint_names_[i]] = 0.0;

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle(joint_names_[i],
                                                          &joint_positions_states_[joint_names_[i]],
                                                          &joint_velocitys_states_[joint_names_[i]],
                                                          &joint_efforts_states_[joint_names_[i]]);
        joint_state_interface_.registerHandle(state_handle);

        //connect and register the position command interface
        hardware_interface::JointHandle position_command_handle(joint_state_interface_.getHandle(joint_names_[i]),
                                                                &joint_position_commands_[joint_names_[i]]);
        position_joint_interface_.registerHandle(position_command_handle);

        //connect and register the velocity command interface
        hardware_interface::JointHandle velocity_command_handle(joint_state_interface_.getHandle(joint_names_[i]),
                                                                &joint_velocity_commands_[joint_names_[i]]);
        velocity_joint_interface_.registerHandle(velocity_command_handle);

        //connect and register the effort command interface
        hardware_interface::JointHandle effort_command_handle(joint_state_interface_.getHandle(joint_names_[i]),
                                                              &joint_effort_commands_[joint_names_[i]]);
        effort_joint_interface_.registerHandle(effort_command_handle);

    }

    registerInterface(&joint_state_interface_   );
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_  );

    new_data_ready_ = false;

    // ROS topic subscribtions for Robotiq Input
    ros::SubscribeOptions robotiqSo =
    ros::SubscribeOptions::create<robotiq_s_model_control::SModel_robot_input>(
        "/robotiq_hands/"+hand_side_+"/SModelRobotInput", 1,boost::bind(&RobotiqHardwareInterface::robotiq_Callback, this, _1),
        ros::VoidPtr(), rosnode->getCallbackQueue());

    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint states, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    robotiqSo.transport_hints = ros::TransportHints().unreliable();

    robotiq_input_sub_ = rosnode->subscribe(robotiqSo);


    // ROS topic advertisers for Robotiq Output
    robotiq_output_pub_ = rosnode->advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/"+hand_side_+"/SModelRobotOutput", 1, true);

    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
}

void RobotiqHardwareInterface::cleanup()
{
    subscriber_spinner_->stop();
}

// Implement robot-specific resouce management
bool RobotiqHardwareInterface::checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const
{
   // this list of controllers can be running at the same time

   return false;
}

void RobotiqHardwareInterface::read(ros::Time time, ros::Duration period)
{
//    ROS_INFO("Reading from %s Robotiq...", hand_side_.c_str());
    joint_positions_states_[joint_names_[0]] = robotiq_input_msg_.gPOA *  0.004784314;         //position for finger A. Do math stuff to figure out joint 1 value
    joint_positions_states_[joint_names_[1]] = robotiq_input_msg_.gPOB *  0.004431373;         //position for finger B. Do math stuff to figure out joint 1 value
    joint_positions_states_[joint_names_[2]] = robotiq_input_msg_.gPOC *  0.004431373;         //position for finger C. Do math stuff to figure out joint 1 value
    joint_positions_states_[joint_names_[3]] = robotiq_input_msg_.gPOS * -0.002012941 + 0.275; //position for scissors finger B.
    joint_positions_states_[joint_names_[4]] = robotiq_input_msg_.gPOS *  0.002012941 - 0.275; //position for scissors finger C.

    joint_efforts_states_[  joint_names_[0]] = robotiq_input_msg_.gCUA; //Current for finger A.
    joint_efforts_states_[  joint_names_[1]] = robotiq_input_msg_.gCUB; //Current for finger B.
    joint_efforts_states_[  joint_names_[2]] = robotiq_input_msg_.gCUC; //Current for finger C.
    joint_efforts_states_[  joint_names_[3]] = robotiq_input_msg_.gCUS; //Current for scissors finger B.
    joint_efforts_states_[  joint_names_[4]] = robotiq_input_msg_.gCUS; //Current for scissors finger C.
}

void RobotiqHardwareInterface::write(ros::Time time, ros::Duration period)
{
//    ROS_INFO("Writing to %s Robotiq...", hand_side_.c_str());
    //Converting Position Request
    robotiq_output_msg_.rPRA = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[0]]  * RAD_TO_BYTE)   ,float(255.0)));
    robotiq_output_msg_.rPRB = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[1]]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_output_msg_.rPRC = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[2]]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_output_msg_.rPRS = std::max(float(0.0), std::min(float((joint_position_commands_[joint_names_[3]]) * SPR_TO_BYTE)   ,float(255.0)));
    //Converting Speed request
    robotiq_output_msg_.rSPA = std::max(float(0.0), std::min(float(joint_velocity_commands_[ joint_names_[0]])                  ,float(255.0)));
    robotiq_output_msg_.rSPB = std::max(float(0.0), std::min(float(joint_velocity_commands_[ joint_names_[1]])                  ,float(255.0)));
    robotiq_output_msg_.rSPC = std::max(float(0.0), std::min(float(joint_velocity_commands_[ joint_names_[2]])                  ,float(255.0)));
    robotiq_output_msg_.rSPS = std::max(float(0.0), std::min(float(joint_velocity_commands_[ joint_names_[3]])                  ,float(255.0)));
    //Converting Force Request
    robotiq_output_msg_.rFRA = std::max(float(0.0), std::min(float(joint_effort_commands_[   joint_names_[0]] * PER_TO_BYTE)    ,float(255.0)));
    robotiq_output_msg_.rFRB = std::max(float(0.0), std::min(float(joint_effort_commands_[   joint_names_[1]] * PER_TO_BYTE)    ,float(255.0)));
    robotiq_output_msg_.rFRC = std::max(float(0.0), std::min(float(joint_effort_commands_[   joint_names_[2]] * PER_TO_BYTE)    ,float(255.0)));
    robotiq_output_msg_.rFRS = std::max(float(0.0), std::min(float(joint_effort_commands_[   joint_names_[3]] * PER_TO_BYTE)    ,float(255.0)));

    robotiq_output_pub_.publish(robotiq_output_msg_);
}

} //end namespace RobotiqHardwareInterface

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "robotiq_ros_controller");
        ros::NodeHandle nh_("~");

        double control_rate;
        nh_.param("control_rate", control_rate, 125.0);


        RobotiqHardwareInterface::RobotiqHardwareInterface robotiq_hw;

        controller_manager::ControllerManager cm(&robotiq_hw);

        ros::AsyncSpinner spinner(4);
        spinner.start();

        ros::Rate rate(control_rate);

        ros::Time last_time = ros::Time::now();

        while (ros::ok())
        {
            //ROS_INFO("in main loop");
            rate.sleep();
            ros::spinOnce();

            ros::Time current_time = ros::Time::now();
            ros::Duration elapsed_time = current_time - last_time;
            last_time = current_time;

            if(robotiq_hw.new_data_ready_)
            {
                robotiq_hw.read(current_time, elapsed_time);
                cm.update(current_time, elapsed_time);
                robotiq_hw.write(current_time, elapsed_time);
                robotiq_hw.new_data_ready_ = false;
            }
        }

        robotiq_hw.cleanup();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
