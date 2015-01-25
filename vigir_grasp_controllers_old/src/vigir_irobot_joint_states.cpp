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

#include <unistd.h>

//#include "flor_grasp_controllers/flor_irobot_joint_states.h"
#include "sensor_msgs/JointState.h"
#include "handle_msgs/HandleSensors.h"
#include "handle_msgs/HandleControl.h"

ros::Publisher irobot_states_pub_;
ros::Publisher ocs_grasp_pub_;
ros::Publisher zeros_pub_;
ros::Publisher calibration_pub_;
std::string hand_;

// globals
sensor_msgs::JointState states_;
handle_msgs::HandleSensors local_msg_;
std::string hand_name_;
std::vector<int>  finger_timeout_;
std::vector<int>  finger_zero_position_;
std::vector<float> filtered_joint_angles_;
boost::mutex local_msg_mutex_;
bool received_first_hand_data_ = false;

bool receive_commands_ = true;
int connected;
int zero = 6750;

float motor_to_rads = 3200/1.57;
float spread_to_rads = 760/1.57;


void irobotCallback(const handle_msgs::HandleSensors& msg)
{
    {
      boost::mutex::scoped_lock lock (local_msg_mutex_);
      local_msg_ = msg;
      received_first_hand_data_ = true;
    }

    //if(!receive_commands_)
    //    return;
    for(int i = 0; i < 3; i++)
        local_msg_.motorHallEncoder[i] -= finger_zero_position_[i];


    states_.position[4] = local_msg_.fingerSpread / spread_to_rads;
    states_.position[5] = local_msg_.fingerSpread / spread_to_rads;


    float temp_joint_val;
    //Finger 1 positions based on proximal joint sensor in smart fingers
    if(msg.responses[1])
    {
        finger_timeout_[0]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(local_msg_.proximalJointAngle[0]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = local_msg_.motorHallEncoder[0] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states_.position[6] = (proximal - temp_joint_val - .1) * 2;
            if(states_.position[6] > 1.5)
                states_.position[6] = 1.5;
        }
        states_.position[0] = temp_joint_val;
        //states.position[6] = 0;
    }
    else
    {
        finger_timeout_[0] += 1;
        states_.position[0] = filtered_joint_angles_[0];
        states_.position[6] = filtered_joint_angles_[6];
    }

    //Finger 2 positions based on proximal joint sensor in smart fingers
    if(msg.responses[3])
    {
        finger_timeout_[1]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(local_msg_.proximalJointAngle[1]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = local_msg_.motorHallEncoder[1] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states_.position[7] = (proximal - temp_joint_val - .1) * 2;
            if(states_.position[7] > 1.5)
                states_.position[7] = 1.5;
        }
        states_.position[1] = temp_joint_val;
    }
    else
    {
        finger_timeout_[1] += 1;
        states_.position[1] = filtered_joint_angles_[1];
        states_.position[7] = filtered_joint_angles_[7];
    }

    //Finger 3 (thumb) positions based on proximal joint sensor in smart fingers
    if(msg.responses[5])
    {
        finger_timeout_[2]=0;
        //proximal joint to radian conversion
        temp_joint_val = float(local_msg_.proximalJointAngle[2]) * 2.70 / 440.0 ;
        //distal joint angle approximation
        double proximal = local_msg_.motorHallEncoder[2] / motor_to_rads;
        if(proximal > temp_joint_val)
        {
            states_.position[3] = (proximal - temp_joint_val - .1) * 2;
            if(states_.position[3] > 1.5)
                states_.position[3] = 1.5;
        }
        states_.position[2] = temp_joint_val;
    }
    else
    {
        finger_timeout_[2] += 1;
        states_.position[2] = filtered_joint_angles_[2];
        states_.position[3] = filtered_joint_angles_[3];
    }


    // Use motorHallEncoder values only if the proximal joint sensor is not working after a certain timeout period
    if(finger_timeout_[0] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 1");
        states_.position[0] = local_msg_.motorHallEncoder[0] / motor_to_rads;
        states_.position[6] = 0;
        if(states_.position[0] > 2.9)
            states_.position[0] = 2.9;
    }
    if(finger_timeout_[1] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 2");
        states_.position[1] = local_msg_.motorHallEncoder[1] / motor_to_rads;
        states_.position[7] = 0;
        if(states_.position[1] > 2.9)
            states_.position[1] = 2.9;
    }
    if(finger_timeout_[2] > 500)
    {
        //ROS_INFO("Using motor encoder values for finger 3");
        states_.position[2] = local_msg_.motorHallEncoder[2] / motor_to_rads;
        states_.position[3] = 0;
        if(states_.position[2] > 2.9)
            states_.position[2] = 2.9;
    }
      //ROS_INFO("Proximal joint angle: [%d, %d, %d]", h_sensors.proximalJointAngle[0],h_sensors.proximalJointAngle[1],h_sensors.proximalJointAngle[2]);

    for (int i = 0; i < 8 ; i++)
    {
        if(states_.position[i] < 0)
            states_.position[i] = 0;
        filtered_joint_angles_[i] = 0.9 * filtered_joint_angles_[i] + 0.1 * states_.position[i];
        states_.position[i] = filtered_joint_angles_[i];
        //states.position[i] = filtered_finger_angles[i] *.9 + states.position[i] * .1;
        states_.velocity[i] = 0;
        states_.effort[i] = 0;
    }

    //states_.effort[0] = finger_timeout_[0];
    //states_.effort[1] = finger_timeout_[1];
    //states_.effort[2] = finger_timeout_[2];



    // Add this section for publishing hand states

    // Timeout used for determining whether to use smart finger values or generic values
    //states = irobot_publish_jointstates(newmsg, hand_name_, finger_timeout_, filtered_joint_angles_);

    // Save the last finger position to use for filtering the current finger position
    for(int i = 0; i < 8; i++)
        filtered_joint_angles_[i] = states_.position[i];

    for(int i = 0; i < 3; i++)
    {
        //finger_timeout_[i] = states_.effort[i];
        if(finger_timeout_[i] > 600)
            finger_timeout_[i] = 600;
        //states_.effort[i] =0;
        //ROS_INFO("Finger %d Timeout value: %d \n", i, finger_timeout_[i]);
    }

    states_.header.stamp = ros::Time::now();

    irobot_states_pub_.publish(states_);
    ocs_grasp_pub_.publish(states_);

    // If still connected and sending messages, reset timeout timer
    connected = 50;

    return;
}


void calibrateHand()
{
    //HandPacket local_msg_copy;
    handle_msgs::HandleSensors local_msg_copy;
    {
      boost::mutex::scoped_lock lock (local_msg_mutex_);
      local_msg_copy = local_msg_;
    }

    printf("Starting calibration routine\n");
    //receive_commands_ = false;
    handle_msgs::HandleControl cmd;
    std::vector<int> cal_joints;
    cal_joints.resize(3);

    // Open hand a slight amount before calibrating
    for(int i = 0; i < 3; i++)
    {
        cal_joints[i] = local_msg_copy.motorHallEncoder[i] - 1000;

        cmd.type[i] = handle_msgs::HandleControl::POSITION;
        cmd.value[i] = cal_joints[i];
        cmd.valid[i] = true;
    }
    calibration_pub_.publish(cmd);
    sleep(1);


    // Get sensor data
    std::vector<int> current_finger_values;
    std::vector<bool> fingers_stopped;
    std::vector<double> last_motor_current;

    current_finger_values.resize(3);
    fingers_stopped.resize(3);
    last_motor_current.resize(3);

    for(int i = 0; i < 3; i++)
    {
        fingers_stopped[i] = false;
        cal_joints[i] += 50;
        cmd.value[i] = cal_joints[i];
        cmd.type[i] = handle_msgs::HandleControl::POSITION;
        cmd.valid[i] = true;
    }

    // Make sure that finger spread is set to zero
    cmd.value[4] = 0;
    cmd.type[4] = handle_msgs::HandleControl::POSITION;
    cmd.valid[4] = true;

    // Start hand moving for calibration check
    calibration_pub_.publish(cmd);
    sleep(1);

    // Move hands to a closed position checking velocity for stalled condition
    while(!fingers_stopped[0] || !fingers_stopped[1] || !fingers_stopped[2])
    {
        ros::spinOnce();
        {
          boost::mutex::scoped_lock lock (local_msg_mutex_);
          local_msg_copy = local_msg_;
        }

        // Get latest motor current values
        for(int j = 0; j < 3; j++)
        {
            last_motor_current[j] = 0;
        }

        for(int i =0; i < 10; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                last_motor_current[j] += local_msg_copy.motorCurrent[j];
            }
            usleep(1000*30);
        }

        // If motor current exceeds a threshold, consider it stalled (stopped)
        if(last_motor_current[0] > 1.25)
            fingers_stopped[0] = true;
        if(last_motor_current[1] > 1.25)
            fingers_stopped[1] = true;
        if(last_motor_current[2] > 1.25)
            fingers_stopped[2] = true;


        for(int i = 0; i < 3; i++)
        {
            //ROS_INFO("finger %d current value: %f ", i, last_motor_current[i]);
            if(!fingers_stopped[i])
            {
                cal_joints[i] += 100;
                cmd.value[i] = cal_joints[i];
                cmd.type[i] = handle_msgs::HandleControl::POSITION;
                cmd.valid[i] = true;
            }
            else
            {
                cmd.value[i] = 0;
                cmd.type[i] = handle_msgs::HandleControl::VELOCITY;
                cmd.valid[i] = true;
            }
        }
        calibration_pub_.publish(cmd);
    }

    sensor_msgs::JointState cal_zeros;
    cal_zeros.position.resize(3);

    // Hand is uniformly closed, open hand to zero position
    for(int i = 0; i < 3; i++)
    {
        current_finger_values[i] = local_msg_copy.motorHallEncoder[i];
        finger_zero_position_[i] = current_finger_values[i] - zero;  // for use WITH finger spacers
        cal_zeros.position[i] = finger_zero_position_[i];
        //finger_zero_position_[i] = current_finger_vallocal_msg_ues[i] - 6750;  // for use WITHOUT finger spacers
        //ROS_INFO("current finger values: %d", current_finger_values[i]);
        //ROS_INFO("current finger ZERO values: %d", finger_zero_position_[i]);

        cmd.type[i] = handle_msgs::HandleControl::POSITION;
        cmd.value[i] = finger_zero_position_[i] + 6000;
        cmd.valid[i] = true;
    }
    calibration_pub_.publish(cmd);

    cal_zeros.header.stamp = ros::Time::now();
    zeros_pub_.publish(cal_zeros);

    // Wait until fingers are open
    sleep(5);
    receive_commands_ = true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flor_irobot_joint_states");

    //void irobotCallback(const handle_msgs::HandleSensors);
    void calibrateHand();

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.getParam("zero", zero);
    nhp.getParam("hand", hand_);
    ROS_INFO("hand type: %s, zero calibration value: %d\n", hand_.c_str(), zero);

    if(hand_ == "right")
        hand_name_ = "r_hand";
    else
        hand_name_ = "l_hand";

    states_.name.resize(8);
    states_.position.resize(8);
    states_.velocity.resize(8);
    states_.effort.resize(8);

    states_.name[0] = hand_ + "_f0_j1";
    states_.name[1] = hand_ + "_f1_j1";
    states_.name[2] = hand_ + "_f2_j1";
    states_.name[3] = hand_ + "_f2_j2";
    states_.name[4] = hand_ + "_f0_j0";
    states_.name[5] = hand_ + "_f1_j0";
    states_.name[6] = hand_ + "_f0_j2";
    states_.name[7] = hand_ + "_f1_j2";

    finger_timeout_.resize(3);
    finger_zero_position_.resize(3);
    filtered_joint_angles_.resize(8);


    //ros::Publisher
    irobot_states_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
    ocs_grasp_pub_ = nh.advertise<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/joint_states", 1, true);
    zeros_pub_ = nh.advertise<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/calibrated_zeros", 1, true);
    calibration_pub_ = nh.advertise<handle_msgs::HandleControl>("/" + hand_ + "_hand/control", 1);

    ros::Subscriber sub1 = nh.subscribe("/" + hand_ + "_hand/sensors/raw", 1, irobotCallback);

    ros::spinOnce();

    while (!received_first_hand_data_){
        ROS_INFO("Have not received hand data yet, waiting");
        ros::spinOnce();
        usleep(1000*100);
    }
    //ros::Subscriber sub2 = nh.subscribe("/handle/calibrate", 1, calibrate_callback);
    calibrateHand();

    ros::spin();
    return 0;
}
