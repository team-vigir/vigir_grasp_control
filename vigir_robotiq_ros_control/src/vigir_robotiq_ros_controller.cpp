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
#include <vigir_robotiq_ros_control/vigir_robotiq_ros_controller.h>
#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPREAD_RAD     0.28   //Radians range of the spread fingers
#define BYTE_TO_SPR    (SPREAD_RAD/255.0)
#define SPR_TO_BYTE    (1/BYTE_TO_SPR)
#define SPR_ZERO       (BYTE_TO_SPR * 137)
#define PER_TO_BYTE    2.55

namespace RobotiqHardwareInterface
{

void RobotiqHardwareInterface::robotiq_Callback(const robotiq_s_model_control::SModel_robot_input::ConstPtr &msg)
{
    if(msg){
        new_data_ready_              = true;
        robotiq_input_msg_           = *msg;        
    }
}

void RobotiqHardwareInterface::robotiq_tactile_Callback(const flor_grasp_msgs::Tactile::ConstPtr &msg)
{
    if(msg){
        last_tactile_msg_ = *msg;
    }
}

RobotiqHardwareInterface::RobotiqHardwareInterface()
{
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    nh.setCallbackQueue(&subscriber_queue_);

    private_nh.param<std::string>("hand_side", hand_side_, std::string("left"));
    private_nh.param<std::string>("hand_name", hand_name_, std::string("l_hand"));

    std::string joint_param  = "joint_names";
    std::string link_param   = "link_names";
    std::string sensor_param = "sensor_array";

    XmlRpc::XmlRpcValue joint_names_xmlrpc;
    if (private_nh.hasParam(joint_param))
        private_nh.getParam(joint_param, joint_names_xmlrpc);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", joint_param.c_str());
        throw std::invalid_argument(joint_param);
    }

    XmlRpc::XmlRpcValue link_names_xmlrpc;
    if (private_nh.hasParam(link_param))
        private_nh.getParam(link_param, link_names_xmlrpc);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", link_param.c_str());
        throw std::invalid_argument(link_param);
    }

    XmlRpc::XmlRpcValue sensor_array_xmlrpc;
    if (private_nh.hasParam(sensor_param))
        private_nh.getParam(sensor_param, sensor_array_xmlrpc);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", sensor_param.c_str());
        throw std::invalid_argument(sensor_param);
    }

    robotiq_activation_time_ = 17.0;

    // which file are we reading

    if (private_nh.hasParam("robotiq_activation_time"))
        private_nh.param<double>("robotiq_activation_time", robotiq_activation_time_, 16);
    else
    {
        ROS_ERROR("Parameter robotiq_activation_time not set, shutting down node...");
        throw std::invalid_argument("robotiq_activation_time");
    }

    ROS_INFO("Robotiq Hand will activate in %f seconds", robotiq_activation_time_);

    //Hand Status Initialization
    hand_status_.joint_states.name.resize(joint_names_xmlrpc.size());
    hand_status_.joint_states.position.resize(joint_names_xmlrpc.size());
    hand_status_.joint_states.velocity.resize(joint_names_xmlrpc.size());
    hand_status_.joint_states.effort.resize(joint_names_xmlrpc.size());
    hand_status_.joint_status.resize(joint_names_xmlrpc.size());
    hand_status_.link_states.name.resize(link_names_xmlrpc.size());
    hand_status_.link_states.tactile_array.resize(link_names_xmlrpc.size()); //Number of links with tactile arrays
    ROS_INFO("%s Robotiq joint links number: %d",hand_side_.c_str(),(int)hand_status_.link_states.name.size());

    int total_tactile=0;

    for(int i=0;i<link_names_xmlrpc.size();i++){
        hand_status_.link_states.name[i] = hand_side_ + "_" + static_cast<std::string>(link_names_xmlrpc[i]);  //Name of link
        hand_status_.link_states.tactile_array[i].pressure.resize(static_cast<int>(sensor_array_xmlrpc[i]));  //Number of tactile sensors per array
        ROS_INFO("Link name %s added to %s Robotiq with %d tactile sensors",hand_status_.link_states.name[i].c_str(),
                                                                            hand_side_.c_str(),
                                                                            (int)hand_status_.link_states.tactile_array[i].pressure.size());
        total_tactile += static_cast<int>(sensor_array_xmlrpc[i]);
    }

    last_tactile_msg_.pressure.resize(total_tactile);
    for(int i=0;i<total_tactile;i++)
        last_tactile_msg_.pressure[i]=0.0;

    for (int i=0; i<joint_names_xmlrpc.size(); i++)
    {
        joint_names_.push_back(hand_side_ + "_" + static_cast<std::string>(joint_names_xmlrpc[i]));
        hand_status_.joint_states.name[i] = hand_side_ + "_" + static_cast<std::string>(joint_names_xmlrpc[i]);
        ROS_INFO("Joint name %s added to %s Robotiq",joint_names_[i].c_str() ,hand_side_.c_str());
    }

    ROS_INFO("%s Robotiq joint names size: %d",hand_side_.c_str(),(int)joint_names_.size());


    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
        joint_position_commands_[    joint_names_[i]] = 0.0;
        last_joint_positions_states_[joint_names_[i]] = 0.0;
        joint_positions_states_[     joint_names_[i]] = 0.0;
        joint_velocitys_states_[     joint_names_[i]] = 0.0;
        joint_efforts_states_[       joint_names_[i]] = 0.0;

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

    }

    registerInterface(&joint_state_interface_   );
    registerInterface(&position_joint_interface_);

    new_data_ready_ = false;
    first_time_     = true;

    // ROS topic subscriber for Robotiq Input
    robotiq_input_sub_ = nh.subscribe("/robotiq_hands/"+hand_name_+"/SModelRobotInput",   1, &RobotiqHardwareInterface::robotiq_Callback,  this);

    // ROS topic subscriber for Robotiq Tactile Input
    tactile_sub_ = nh.subscribe("/robotiq_hands/"+hand_name_+"/hand_contacts",   1, &RobotiqHardwareInterface::robotiq_tactile_Callback,  this);

    // ROS topic advertisers for Robotiq Output
    robotiq_output_pub_ = nh.advertise<robotiq_s_model_control::SModel_robot_output>("/robotiq_hands/"+hand_name_+"/SModelRobotOutput", 1, true);

    // ROS topic advertisers for Robotiq Hand Status
    hand_status_pub_ = nh.advertise<flor_grasp_msgs::HandStatus>("/grasp_control/"+hand_name_+"/hand_status", 1, true);

    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();
}

void RobotiqHardwareInterface::InitializeRobotiq(){
    //INITIALIZE ROBOTIQ HAND MSG
    robotiq_output_msg_.rATR = 0; //NO Automatic release
    robotiq_output_msg_.rGTO = 0; //Got to position
    robotiq_output_msg_.rICF = 0; //Individual control finger
    robotiq_output_msg_.rICS = 0; //Individual scissor control
    robotiq_output_msg_.rMOD = 0; //Ignore modes due ICS activated

    //Position Request
    robotiq_output_msg_.rPRA = 0; //FINGER OPEN
    robotiq_output_msg_.rPRB = 0; //FINGER OPEN
    robotiq_output_msg_.rPRC = 0; //FINGER OPEN
    robotiq_output_msg_.rPRS = 137; //FINGER CYLINDRICAL
    //Speed Request
    robotiq_output_msg_.rSPA = 0; //FINGER SPEED
    robotiq_output_msg_.rSPB = 0; //FINGER SPEED
    robotiq_output_msg_.rSPC = 0; //FINGER SPEED
    robotiq_output_msg_.rSPS = 0; //FINGER SPEED
    //Force Request
    robotiq_output_msg_.rFRA = 0; //FINGER FORCE
    robotiq_output_msg_.rFRB = 0; //FINGER FORCE
    robotiq_output_msg_.rFRC = 0; //FINGER FORCE
    robotiq_output_msg_.rFRS = 0; //FINGER FORCE

    //First Reset Hand
    robotiq_output_msg_.rACT = 0; //Set to ZERO to reset
    robotiq_output_pub_.publish(robotiq_output_msg_);

    ros::Duration(0.5).sleep(); //SLEEP WAITING FOR THE HAND TO ACTIVATE

    //ACTIVATE ROBOTIQ HAND
    robotiq_output_msg_.rSPA = 255; //FINGER SPEED
    robotiq_output_msg_.rFRA = 150; //FINGER FORCE
    robotiq_output_msg_.rGTO = 1; //Got to position
    robotiq_output_msg_.rACT = 1; //Set to ONE to activate
    robotiq_output_pub_.publish(robotiq_output_msg_);

    ROS_WARN("ACTIVATING %s Robotiq... ",hand_side_.c_str() );

    ros::Duration(robotiq_activation_time_).sleep(); //SLEEP WAITING FOR THE HAND TO ACTIVATE

    ROS_WARN("%s Robotiq ACTIVATED",hand_side_.c_str() );

    robotiq_output_msg_.rICF = 1; //Individual control finger
    robotiq_output_msg_.rICS = 1; //Individual scissor control
    robotiq_output_pub_.publish(robotiq_output_msg_);

    ros::Duration(0.5).sleep(); //SLEEP

    //SPEED REQUEST
    robotiq_output_msg_.rSPA = 255;
    robotiq_output_msg_.rSPB = 255;
    robotiq_output_msg_.rSPC = 255;
    robotiq_output_msg_.rSPS = 255;
    //Force Request
    robotiq_output_msg_.rFRA = 255;
    robotiq_output_msg_.rFRB = 255;
    robotiq_output_msg_.rFRC = 255;
    robotiq_output_msg_.rFRS = 255;

    robotiq_output_pub_.publish(robotiq_output_msg_);


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
    for(int i=0;i<joint_names_.size();i++){
        last_joint_positions_states_[joint_names_[i]] =  joint_positions_states_[joint_names_[i]];
        switch (i) {
        case 0: //Finger A
            hand_status_.joint_status[0]          = robotiq_input_msg_.gDTA;
            hand_status_.joint_states.position[0] = joint_positions_states_[joint_names_[0]] = robotiq_input_msg_.gPOA *  0.004784314;         //position of finger A. Do math stuff to figure out joint 1 value
            hand_status_.joint_states.effort[0]   = joint_efforts_states_[  joint_names_[0]] = robotiq_input_msg_.gCUA; //Current of finger A.
            break;
        case 1: //Finger B
            hand_status_.joint_status[1]          = robotiq_input_msg_.gDTB;
            hand_status_.joint_states.position[1] = joint_positions_states_[joint_names_[1]] = robotiq_input_msg_.gPOB *  0.004431373;         //position of finger B. Do math stuff to figure out joint 1 value
            hand_status_.joint_states.effort[1]   = joint_efforts_states_[  joint_names_[1]] = robotiq_input_msg_.gCUB; //Current of finger B.
            break;
        case 2: //Finger C
            hand_status_.joint_status[2]          = robotiq_input_msg_.gDTC;
            hand_status_.joint_states.position[2] = joint_positions_states_[joint_names_[2]] = robotiq_input_msg_.gPOC *  0.004431373;         //position of finger C. Do math stuff to figure out joint 1 value
            hand_status_.joint_states.effort[2]   = joint_efforts_states_[  joint_names_[2]] = robotiq_input_msg_.gCUC; //Current of finger C.
            break;
        case 3: //Finger BC Spread
            hand_status_.joint_status[3]          = robotiq_input_msg_.gDTS;
            hand_status_.joint_status[4]          = robotiq_input_msg_.gDTS;
            hand_status_.joint_states.position[3] = joint_positions_states_[joint_names_[3]] = robotiq_input_msg_.gPOS *  BYTE_TO_SPR - SPR_ZERO; //position of scissors finger B.
            hand_status_.joint_states.position[4] = joint_positions_states_[joint_names_[4]] = robotiq_input_msg_.gPOS * -BYTE_TO_SPR + SPR_ZERO; //position of scissors finger C.
            hand_status_.joint_states.effort[3]   = joint_efforts_states_[  joint_names_[3]] = robotiq_input_msg_.gCUS; //Current of scissors finger B.
            hand_status_.joint_states.effort[4]   = joint_efforts_states_[  joint_names_[4]] = robotiq_input_msg_.gCUS; //Current of scissors finger C.
            break;
        default:
            break;
        }
        if(period.toSec() != 0.0){
            hand_status_.joint_states.velocity[  i]  =
            joint_velocitys_states_[joint_names_[i]] = (joint_positions_states_[joint_names_[i]] -
                                                   last_joint_positions_states_[joint_names_[i]])/period.toSec();
        }
    }
    hand_status_.header.stamp    = ros::Time::now();
    hand_status_.hand_status     = robotiq_input_msg_.gIMC;

    if(robotiq_input_msg_.gFLT  != 0)
        hand_status_.hand_status = robotiq_input_msg_.gFLT;

    for(int link_idx=0, sensor_idx=0;link_idx<hand_status_.link_states.tactile_array.size();link_idx++)
        for(int array_idx=0;array_idx<hand_status_.link_states.tactile_array[link_idx].pressure.size();array_idx++, sensor_idx++)
            hand_status_.link_states.tactile_array[link_idx].pressure[array_idx] = last_tactile_msg_.pressure[sensor_idx];

    hand_status_pub_.publish(hand_status_);
}

void RobotiqHardwareInterface::write(ros::Time time, ros::Duration period)
{
    for(int i=0;i<joint_names_.size();i++)
    {
        if(joint_position_commands_[joint_names_[i]] != joint_position_commands_[joint_names_[i]] )
        {
            ROS_ERROR("Position command for joint %d \"%s\", is a NAN-> %f!!!!! setting to 0.0",i,joint_names_[i].c_str(), joint_position_commands_[joint_names_[i]]);
            joint_position_commands_[joint_names_[i]] = 0.0;
        }
    }

    robotiq_output_msg_.rPRA = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[0]]  * RAD_TO_BYTE)   ,float(255.0)));
    robotiq_output_msg_.rPRB = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[1]]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_output_msg_.rPRC = std::max(float(0.0), std::min(float(joint_position_commands_[ joint_names_[2]]  * RAD_BC_TO_BYTE),float(255.0)));
    robotiq_output_msg_.rPRS = std::max(float(0.0), std::min(float(((joint_position_commands_[joint_names_[3]]) + SPR_ZERO) * SPR_TO_BYTE)   ,float(255.0)));
//    ROS_INFO("Robotiq %s ROS Control is writing command: %f  to output: %d",hand_side_.c_str() ,joint_position_commands_[joint_names_[3]],robotiq_output_msg_.rPRS);

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

        //Activate Robotiq hand
        robotiq_hw.InitializeRobotiq();

        ros::Rate rate(control_rate);

        ros::Time last_time = ros::Time::now();

        while(!robotiq_hw.new_data_ready_);

        ROS_INFO("New data ready, starting robotiq ros controller loop");

        while (ros::ok())
        {
                ros::Time current_time = ros::Time::now();
                ros::Duration elapsed_time = current_time - last_time;
                while (ros::Duration(0.0) == elapsed_time)
                {   // Should only be an issue in simulation
                    current_time = ros::Time::now();
                    elapsed_time = current_time - last_time;
                }
                last_time = current_time;

                robotiq_hw.read(current_time, elapsed_time);
                cm.update(current_time, elapsed_time);
                robotiq_hw.write(current_time, elapsed_time);

                rate.sleep();
        }

        robotiq_hw.cleanup();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception! ");
        return -1;
    }

    return 0;
}
