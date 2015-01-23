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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <vector>

#include <sensor_msgs/JointState.h>
#include <handle_msgs/HandleSensors.h>




sensor_msgs::JointState irobot_publish_jointstates(handle_msgs::HandleSensors&  h_sensors, std::string hand_name, std::vector<int> finger_timeout, std::vector<float> filtered_finger_angles)
{
    sensor_msgs::JointState states;

    float motor_to_rads;
    float spread_to_rads;

    motor_to_rads = 3200/1.57;
    spread_to_rads = 760/1.57;
    std::string hand;


    if (hand_name == "10.66.171.23")
        hand = "right";
    else
        hand = "left";

    states.header = h_sensors.header;
    states.header.stamp = ros::Time::now();

    states.name.resize(8);
    states.position.resize(8);
    states.velocity.resize(8);
    states.effort.resize(8);

    states.name[0] = hand + "_f0_j1";
    states.name[1] = hand + "_f1_j1";
    states.name[2] = hand + "_f2_j1";
    states.name[3] = hand + "_f2_j2";
    states.name[4] = hand + "_f0_j0";
    states.name[5] = hand + "_f1_j0";
    states.name[6] = hand + "_f0_j2";
    states.name[7] = hand + "_f1_j2";

    // Use motorHallEncoder values only if the proximal joint sensor is not working after a certain timeout period
    if(finger_timeout[0] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 1");
        states.position[0] = h_sensors.motorHallEncoder[0] / motor_to_rads;
        states.position[6] = 0;
        if(states.position[0] > 2.9)
            states.position[0] = 2.9;
    }
    if(finger_timeout[1] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 2");
        states.position[1] = h_sensors.motorHallEncoder[1] / motor_to_rads;
        states.position[7] = 0;
        if(states.position[1] > 2.9)
            states.position[1] = 2.9;
    }
    if(finger_timeout[2] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 3");
        states.position[2] = h_sensors.motorHallEncoder[2] / motor_to_rads;
        states.position[3] = 0;
        if(states.position[2] > 2.9)
            states.position[2] = 2.9;
    }

    states.position[4] = h_sensors.fingerSpread / spread_to_rads;
    states.position[5] = h_sensors.fingerSpread / spread_to_rads;


    float temp_joint_val;
    //Finger 1 positions based on proximal joint sensor in smart fingers
    if(h_sensors.responses[1])
    {
        finger_timeout[0]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(h_sensors.proximalJointAngle[0]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = h_sensors.motorHallEncoder[0] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states.position[6] = (proximal - temp_joint_val - .1) * 2;
            if(states.position[6] > 1.5)
                states.position[6] = 1.5;
        }
        states.position[0] = temp_joint_val;
        //states.position[6] = 0;
    }
    else
    {
        finger_timeout[0] += 1;
        states.position[0] = filtered_finger_angles[0];
        states.position[6] = filtered_finger_angles[6];
    }

    //Finger 2 positions based on proximal joint sensor in smart fingers
    if(h_sensors.responses[3])
    {
        finger_timeout[1]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(h_sensors.proximalJointAngle[1]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = h_sensors.motorHallEncoder[1] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states.position[7] = (proximal - temp_joint_val - .1) * 2;
            if(states.position[7] > 1.5)
                states.position[7] = 1.5;
        }
        states.position[1] = temp_joint_val;
    }
    else
    {
        finger_timeout[1] += 1;
        states.position[1] = filtered_finger_angles[1];
        states.position[7] = filtered_finger_angles[7];
    }

    //Finger 3 (thumb) positions based on proximal joint sensor in smart fingers
    if(h_sensors.responses[5])
    {
        finger_timeout[2]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(h_sensors.proximalJointAngle[2]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = h_sensors.motorHallEncoder[2] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states.position[3] = (proximal - temp_joint_val - .1) * 2;
            if(states.position[3] > 1.5)
                states.position[3] = 1.5;
        }
        states.position[2] = temp_joint_val;
    }
    else
    {
        finger_timeout[2] += 1;
        states.position[2] = filtered_finger_angles[2];
        states.position[3] = filtered_finger_angles[3];
    }

      //ROS_INFO("Proximal joint angle: [%d, %d, %d]", h_sensors.proximalJointAngle[0],h_sensors.proximalJointAngle[1],h_sensors.proximalJointAngle[2]);

    for (int i = 0; i < 8 ; i++)
    {
        //states.position[i] = filtered_finger_angles[i] *.9 + states.position[i] * .1;
        states.velocity[i] = 0;
        states.effort[i] = 0;
    }

    states.effort[0] = finger_timeout[0];
    states.effort[1] = finger_timeout[1];
    states.effort[2] = finger_timeout[2];

    return states;

};



