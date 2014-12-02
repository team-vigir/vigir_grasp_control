//=================================================================================================
// Copyright (c) 2013, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the names of TU Darmstadt, Virginia Tech, Oregon State, nor TORC Robotics,
//       nor the names of its contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.

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
#include <iostream>
#include <fstream>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <robotiq_s_model_control/SModel_robot_input.h>
#include <sensor_msgs/JointState.h>

#define NUM_JOINTS 11

ros::Publisher finger_joint_states_;
ros::Publisher ocs_grasp_pub_;
std::string    hand_;


void robotiq_Callback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg)
{
    sensor_msgs::JointState robotiq_states;

    robotiq_states.name.resize(NUM_JOINTS);
    robotiq_states.position.resize(NUM_JOINTS);
    robotiq_states.velocity.resize(NUM_JOINTS);
    robotiq_states.effort.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        robotiq_states.position[i] = 0;
        robotiq_states.velocity[i] = 0;
        robotiq_states.effort[i]   = 0;
    }

    //hardcoded joint names. Might be good to make into a loop for more modular code
    robotiq_states.name[0]  = hand_ + "_f0_j1";
    robotiq_states.name[1]  = hand_ + "_f1_j1";
    robotiq_states.name[2]  = hand_ + "_f2_j1";
    robotiq_states.name[3]  = hand_ + "_f1_j0";
    robotiq_states.name[4]  = hand_ + "_f2_j0";
    robotiq_states.name[5]  = hand_ + "_f0_j2";
    robotiq_states.name[6]  = hand_ + "_f1_j2";
    robotiq_states.name[7]  = hand_ + "_f2_j2";
    robotiq_states.name[8]  = hand_ + "_f0_j3";
    robotiq_states.name[9]  = hand_ + "_f1_j3";
    robotiq_states.name[10] = hand_ + "_f2_j3";

    // Since finger A does not have a 0 joint we start at joint 1 first
    robotiq_states.position[0] = msg->gPOA * 0.004784314; //position for finger A. Do math stuff to figure out joint 1 value
    robotiq_states.position[1] = msg->gPOB * 0.004431373; //position for finger B. Do math stuff to figure out joint 1 value
    robotiq_states.position[2] = msg->gPOC * 0.004431373; //position for finger C. Do math stuff to figure out joint 1 value

    robotiq_states.position[3] = msg->gPOS * -0.002012941 + 0.275; //position for finger B.
    robotiq_states.position[4] = msg->gPOS *  0.002012941 - 0.275; //position for finger C.

//    robotiq_states_.position[5] = msg->gPOA; //position for finger A. Do math stuff to figure out joint 2 value
//    robotiq_states_.position[6] = msg->gPOB; //position for finger B. Do math stuff to figure out joint 2 value
//    robotiq_states_.position[7] = msg->gPOC; //position for finger C. Do math stuff to figure out joint 2 value

//    robotiq_states_.position[8] = msg->gPOA; //position for finger A. Do math stuff to figure out joint 3 value
//    robotiq_states_.position[9] = msg->gPOB; //position for finger B. Do math stuff to figure out joint 3 value
//    robotiq_states_.position[10] = msg->gPOC;//position for finger C. Do math stuff to figure out joint 3 value


    // hand_status_ Publisher

    //   hand_status_.echoPosition[0] = msg->gPRA; // [0,255] value requested for finger A
    //   hand_status_.echoPosition[1] = msg->gPRB; // [0,255] value requested for finger B
    //   hand_status_.echoPosition[2] = msg->gPRC; // [0,255] value requested for finger C

    //   hand_status_.currentValue[0] = msg->gCUA; // Current of finger A 0.1 * Current (value in mA)
    //   hand_status_.currentValue[1] = msg->gCUB; // Current of finger B 0.1 * Current (value in mA)
    //   hand_status_.currentValue[2] = msg->gCUC; // Current of finger C 0.1 * Current (value in mA)

    //   hand_status_.status = msg->gFLT;  // Values given that tell us when a fault occurs
    //                                     // 0 = No fault has occured
    //                                     // 5 = Action delayed. Activate/Reactive before continuing
    //                                     // 6 = Action delayed. Mode change must occur before continuing
    //                                     // 7 = Activation bit was not set. Please set before continuing
    //                                     // 9 = Communication chip is not ready. May be activating.
    //                                     // 10 = Changing Mode Fault. Interference with Scissor mode < 20 seconds
    //                                     // 11 = Automatic Release in progress
    //                                     // 13 = Activation fault. Verify no interference or other errors
    //                                     // 14 = Changing Mode Fault. Interference with Scissor mode > 20 seconds
    //                                     // 15 = Automatic Release complete. Reset and activation required.


    //   hand_status_.mode = msg->gMOD; // Current Mode of hand 0 = Basic, 1 = Wide, 2 = Pinch, 3 = Scissor

    //   hand_status_.moving = msg->gGTO; // Echo of rGTO 0 = Stopped, 1 = Moving to requested position

    //   hand_status_.changeState = msg->gIMC; // 0 = Gripper is in reset or auto release state. Check gFLT
    //                                         // 1 = Mode change is in progress.
    //                                         // 2 = Activation is in progress
    //                                         // 3 = Activation and mode change are complete.

    //   hand_status_.stopped = msg->gSTA; // 0 = Gripper is in motion as requested. (Only helpful iff gGTO == 1)
    //                                     // 1 = Gripper is stopped. All fingers stopped early.
    //                                     // 2 = Gripper is stopped. 1 or 2 fingers stopped early
    //                                     // 3 = Gripper is stopped. All fingers reached final positions

    //   hand_status_.fingerA = msg->gDTA; // 0 = Finger A is in motion. (Only helpful iff gGTO == 1)
    //                                     // 1 = Finger A has stopped due to contact while closing
    //                                     // 2 = Finger A has stopped due to contact while opening
    //                                     // 3 = finger A has reached the requested destination

    //   hand_status_.fingerB = msg->gDTB; // 0 = Finger B is in motion. (Only helpful iff gGTO == 1)
    //                                     // 1 = Finger B has stopped due to contact while closing
    //                                     // 2 = Finger B has stopped due to contact while opening
    //                                     // 3 = finger B has reached the requested destination

    //   hand_status_.fingerC = msg->gDTC; // 0 = Finger C is in motion. (Only helpful iff gGTO == 1)
    //                                     // 1 = Finger C has stopped due to contact while closing
    //                                     // 2 = Finger C has stopped due to contact while opening
    //                                     // 3 = finger C has reached the requested destination

    //   hand_status_.scissor = msg->gDTS; // 0 = Scissor is in motion. (Only helpful iff gGTO == 1)
    //                                     // 1 = Scissor has stopped due to contact while closing
    //                                     // 2 = Scissor has stopped due to contact while opening
    //                                     // 3 = Scissor has reached the requested destination


    robotiq_states.header.stamp = ros::Time::now();
    finger_joint_states_.publish(robotiq_states);
    ocs_grasp_pub_.publish(robotiq_states);
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vigir_robotiq_joint_states");

    void robotiq_Callback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg);

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");


    std::string hand_name_;

    ros::NodeHandle("~").getParam("hand", hand_);
    ROS_INFO("hand type: %s", hand_.c_str());

    if(hand_ == "right")
        hand_name_ = "r_hand";
    else
        hand_name_ = "l_hand";

    //ros::Publisher
    finger_joint_states_  = nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
    ocs_grasp_pub_        = nh.advertise<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/joint_states", 1, true);

    ros::Subscriber robotiq_sub_;

    robotiq_sub_= nh.subscribe("/robotiq_hands/"+hand_name_+"/SModelRobotInput", 1, robotiq_Callback);

    ros::spin();
    return 0;
}
