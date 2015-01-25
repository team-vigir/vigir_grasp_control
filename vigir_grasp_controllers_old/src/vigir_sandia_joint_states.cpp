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

#include <sandia_hand_msgs/RawFingerState.h>
#include <sensor_msgs/JointState.h>

ros::Publisher finger_joint_states_;
ros::Publisher ocs_grasp_pub_;
std::string hand_;

void finger_0_Callback(sandia_hand_msgs::RawFingerState msg)
{
    sensor_msgs::JointState sandia_states;
    sandia_states.name.resize(3);
    sandia_states.position.resize(3);
    sandia_states.velocity.resize(3);
    sandia_states.effort.resize(3);

    sandia_states.name[0] = hand_ + "_f0_j0";
    sandia_states.name[1] = hand_ + "_f0_j1";
    sandia_states.name[2] = hand_ + "_f0_j2";

    for (int i = 0; i < 3; i++)
    {
        sandia_states.position[i] = msg.hall_pos[i]   / 1600.0 * 1.57; //2024.0;
    }
	sandia_states.position[0] = msg.hall_pos[0] / 1700 * .79; //sandia_states.position[0] * -1.0;
    sandia_states.position[1] += sandia_states.position[0];
    sandia_states.position[2] = (sandia_states.position[2] - sandia_states.position[0] + sandia_states.position[1])/3.0;

    sandia_states.header.stamp = ros::Time::now();
    finger_joint_states_.publish(sandia_states);
    ocs_grasp_pub_.publish(sandia_states);
    return;
}

void finger_1_Callback(sandia_hand_msgs::RawFingerState msg)
{
    sensor_msgs::JointState sandia_states;
    sandia_states.name.resize(3);
    sandia_states.position.resize(3);
    sandia_states.velocity.resize(3);
    sandia_states.effort.resize(3);

    sandia_states.name[0] = hand_ + "_f1_j0";
    sandia_states.name[1] = hand_ + "_f1_j1";
    sandia_states.name[2] = hand_ + "_f1_j2";

    for (int i = 0; i < 3; i++)
    {
        sandia_states.position[i] = msg.hall_pos[i]  / 1600.0 * 1.57; //2024.0;
    }
	sandia_states.position[0] = msg.hall_pos[0] / 1700 * .79; //sandia_states.position[0] * -1.0;
    sandia_states.position[1] += sandia_states.position[0];
    sandia_states.position[2] = (sandia_states.position[2] - sandia_states.position[0] + sandia_states.position[1])/3.0;

    sandia_states.header.stamp = ros::Time::now();
    finger_joint_states_.publish(sandia_states);
    ocs_grasp_pub_.publish(sandia_states);
    return;
}

void finger_2_Callback(sandia_hand_msgs::RawFingerState msg)
{
    sensor_msgs::JointState sandia_states;
    sandia_states.name.resize(3);
    sandia_states.position.resize(3);
    sandia_states.velocity.resize(3);
    sandia_states.effort.resize(3);

    sandia_states.name[0] = hand_ + "_f2_j0";
    sandia_states.name[1] = hand_ + "_f2_j1";
    sandia_states.name[2] = hand_ + "_f2_j2";

    for (int i = 0; i < 3; i++)
    {
        sandia_states.position[i] = msg.hall_pos[i]  / 1600.0 * 1.57; //2024.0;
    }
	sandia_states.position[0] = msg.hall_pos[0] / 1700 * .79; //sandia_states.position[0] * -1.0;
    sandia_states.position[1] += sandia_states.position[0];
    sandia_states.position[2] = (sandia_states.position[2] - sandia_states.position[0] + sandia_states.position[1])/3.0;

    sandia_states.header.stamp = ros::Time::now();
    finger_joint_states_.publish(sandia_states);
    ocs_grasp_pub_.publish(sandia_states);
    return;
}

void finger_3_Callback(sandia_hand_msgs::RawFingerState msg)
{
    sensor_msgs::JointState sandia_states;
    sandia_states.name.resize(3);
    sandia_states.position.resize(3);
    sandia_states.velocity.resize(3);
    sandia_states.effort.resize(3);

    sandia_states.name[0] = hand_ + "_f3_j0";
    sandia_states.name[1] = hand_ + "_f3_j1";
    sandia_states.name[2] = hand_ + "_f3_j2";

    for (int i = 0; i < 3; i++)
    {
        sandia_states.position[i] = msg.hall_pos[i]  / 1600.0 * 1.57; //2024.0;
    }
	sandia_states.position[0] = msg.hall_pos[0] / 1700 * .79; //sandia_states.position[0] * -1.0;
    sandia_states.position[1] += sandia_states.position[0];
    sandia_states.position[2] = (sandia_states.position[2] - sandia_states.position[0] + sandia_states.position[1])/3.0;

    sandia_states.header.stamp = ros::Time::now();
    finger_joint_states_.publish(sandia_states);
    ocs_grasp_pub_.publish(sandia_states);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flor_sandia_joint_states");

    void finger_0_Callback(sandia_hand_msgs::RawFingerState msg);
    void finger_1_Callback(sandia_hand_msgs::RawFingerState msg);
    void finger_2_Callback(sandia_hand_msgs::RawFingerState msg);
    void finger_3_Callback(sandia_hand_msgs::RawFingerState msg);

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");


    std::string hand_name_;

    ros::NodeHandle("~").getParam("hand", hand_);
    //ros::param::get("hand", hand_);
    ROS_INFO("hand type: %s", hand_.c_str());

    if(hand_ == "right")
        hand_name_ = "r_hand";
    else
        hand_name_ = "l_hand";

    //ros::Publisher
    finger_joint_states_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
    ocs_grasp_pub_ = nh.advertise<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/joint_states", 1, true);


    ros::Subscriber finger_0_sub_;
    ros::Subscriber finger_1_sub_;
    ros::Subscriber finger_2_sub_;
    ros::Subscriber finger_3_sub_;



    //ros::SubscribeOptions f0So =
    //ros::SubscribeOptions::create<sandia_hand_msgs::RawFingerState>
    finger_0_sub_ = nh.subscribe("/sandia_hands/"+hand_name_+"/finger_0/raw_state", 1, finger_0_Callback);
    finger_1_sub_ = nh.subscribe("/sandia_hands/"+hand_name_+"/finger_1/raw_state", 1, finger_1_Callback);
    finger_2_sub_ = nh.subscribe("/sandia_hands/"+hand_name_+"/finger_2/raw_state", 1, finger_2_Callback);
    finger_3_sub_ = nh.subscribe("/sandia_hands/"+hand_name_+"/finger_3/raw_state", 1, finger_3_Callback);


    ros::spin();
    return 0;
}
