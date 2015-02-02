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

#include <iostream>
#include <boost/thread/locks.hpp>
#include <fstream>
#include <algorithm>
#include <unistd.h>

#include <flor_ocs_msgs/RobotStatusCodes.h>

#include <vigir_grasp_controllers_old/vigir_irobot_grasp_controller.h>
#include <handle_msgs/Finger.h>

namespace vigir_grasp_controllers_old{



    VigirIRobotGraspController::VigirIRobotGraspController()
      : VigirGraspController() // explicitly initialize the base class
    {
        // Create tactile sensor variables
        filtered_tactile_msg_.resize(3);
        tactile_zeros_.resize(3);

        for(int i = 0; i < 3; i++)
        {
            filtered_tactile_msg_[i].proximal.resize(12);
            filtered_tactile_msg_[i].distal.resize(10);

            tactile_zeros_[i].proximal.resize(12);
            tactile_zeros_[i].distal.resize(10);
        }

        palm_tactile_zeros_.resize(48);
        filtered_palm_tactile_msg_.resize(48);
        tactile_timeout_counter_.resize(8);

        //local_tactile_msg_=filtered_tactile_msg_;
        //local_palm_tactile_msg_=filtered_palm_tactile_msg_;

        std::string tmp_hand_name;
        // Processed tactile data for feedback to OCS
        if(hand_id_ < 0)
        {
            tmp_hand_name = "left_";
        }
        else
        {
            tmp_hand_name = "right_";
        }

        // Name of links used in URDF model
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[0]/proximal_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[1]/proximal_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[2]/proximal_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[0]/base_rotation_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[1]/base_rotation_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[0]/distal_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[1]/distal_link");
        tactile_feedback_msg_.name.push_back(tmp_hand_name+"finger[2]/distal_link");

        //tactile_feedback_msg_.name.push_back(tmp_hand_name+"f2_j0");

        tactile_feedback_msg_.velocity.resize(8);
        //tactile_feedback_msg_.position.resize(8);
        //tactile_feedback_msg_.effort.resize(8);

        /*
        for(int i = 0; i < 9; i ++)
        {
            tactile_feedback_msg_.effort[i] = 0;
            tactile_feedback_msg_.velocity[i] = 0;
            tactile_feedback_msg_.position[i] = 0;
        }
        */
        zero_velocity_counter_ = 0;

        // only looking at 3 motors
        hand_temperature_counter_.resize(3);
        //for(int i = 0; i < 3; i++)
        //    hand_temperature_counter_[i] = 0;

        publish_counter_ = 0;
        hand_status_.resize(3);

        if(hand_id_ > 0)
        {
          hand_status_[0].status = RobotStatusCodes::status(RobotStatusCodes::RIGHT_FINGER_1_OK, RobotStatusCodes::OK);
          hand_status_[1].status = RobotStatusCodes::status(RobotStatusCodes::RIGHT_FINGER_2_OK, RobotStatusCodes::OK);
          hand_status_[2].status = RobotStatusCodes::status(RobotStatusCodes::RIGHT_FINGER_3_OK, RobotStatusCodes::OK);
        }
        else
        {
          hand_status_[0].status = RobotStatusCodes::status(RobotStatusCodes::LEFT_FINGER_1_OK, RobotStatusCodes::OK);
          hand_status_[1].status = RobotStatusCodes::status(RobotStatusCodes::LEFT_FINGER_2_OK, RobotStatusCodes::OK);
          hand_status_[2].status = RobotStatusCodes::status(RobotStatusCodes::LEFT_FINGER_3_OK, RobotStatusCodes::OK);
        }

    }

    VigirIRobotGraspController::~VigirIRobotGraspController()
    {
        ROS_WARN("Shutting down the iRobot Hand grasping controller ...");
    }


    //////////////////////////////////////////////////////////////////////////
    // IRobot class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirIRobotGraspController::initializeIRobotGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {

        spread_conversion = 760/1.57;
        rad_to_pos_conversion = 3500/1.57; //  3200/1.0
        zero_joint_values_.resize(3);
      // Initialize the generic grasp controller components
      initializeGraspController(nh,nhp);


      // HARDWARE PUBLISHERS
      std::string irobot_hand_name = "right_hand";
	  std::string hand_name = "r_hand";
      if (hand_id_ < 0)
	  {
          irobot_hand_name = "left_hand";
		  hand_name = "l_hand";
	  }

      ROS_INFO("Setup parameters and communications for the iRobot Hand grasp controller plugin for %s hand", irobot_hand_name.c_str());
      joint_commands_pub_ = nh.advertise<handle_msgs::HandleControl>("/"+irobot_hand_name+"/control", 1, true);
      irobot_states_pub_= nh.advertise<sensor_msgs::JointState>("irobot_states", 1, true);
      irobot_status_pub_= nh.advertise<flor_ocs_msgs::OCSRobotStatus>("/"+irobot_hand_name+"/status", 1, true);

      aco_pub_                  = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, true);

      // SIMULATION PUBLISHERS
      //joint_commands_pub_ = nh.advertise<osrf_msgs::JointCommands>("/irobot_hands/"+hand_name_+"/joint_commands", 1, true);
      //irobot_states_pub_= nh.advertise<sensor_msgs::JointState>("irobot_states", 1, true);


      // HARDWARE SUBSCRIPTIONS
      ros::SubscribeOptions jointStatesSo =
      ros::SubscribeOptions::create<handle_msgs::HandleSensors>(
          "/"+irobot_hand_name+"/sensors/raw", 1, boost::bind(&VigirIRobotGraspController::handleSensorsCallback, this, _1),
              ros::VoidPtr(), nh.getCallbackQueue());

      // Because TCP causes bursty communication with high jitter,
      // declare a preference on UDP connections for receiving
      // joint states, which we want to get at a high rate.
      // Note that we'll still accept TCP connections for this topic
      // (e.g., from rospy nodes, which don't support UDP);
      // we just prefer UDP.
      jointStatesSo.transport_hints = ros::TransportHints().unreliable();
      measured_joint_state_sub_ = nh.subscribe(jointStatesSo);

      ros::SubscribeOptions calibrationStatesSo =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/grasp_control/"+hand_name+"/calibrated_zeros", 1, boost::bind(&VigirIRobotGraspController::calibrateCallback, this, _1),
              ros::VoidPtr(), nh.getCallbackQueue());
      calibrationStatesSo.transport_hints = ros::TransportHints().unreliable();
      calibration_sub_ = nh.subscribe(calibrationStatesSo);

      // SOFTWARE SUBSCRITIONS

      // Load the hand specific grasping database
      loadIRobotGraspDatabase(this->filename);

    }


    // Hand specific implementations used by the grasp controller

    void VigirIRobotGraspController::ZeroHandJointCommands()
    {
        for (unsigned i = 0; i < this->joint_names_.size(); ++i)
        {
            this->joint_commands_.type[i] = 0;
            this->joint_commands_.value[i] = 0;
            this->joint_commands_.valid[i] = false;
        }
        this->finger_close_scale_ = 0;
        this->old_joint_commands_ = this->joint_commands_;
        return;
    }

    void VigirIRobotGraspController::processHandSensorData()          // protected by sensor_data and write_data mutex locks
    {
        // Store the ROS message data into local structures for thread safe access
        //if(sensor_data_ready_)
        this->local_handle_msg_ = *(this->last_joint_state_msg_);

        for(int i = 0; i < 3; i++)
            local_handle_msg_.motorHallEncoder[i] -= zero_joint_values_[i];

        // Set hand status from sensor data


        if(publish_counter_ < 100)
        {
            publish_counter_++;
          // only looking at 3 motors
          for(int i = 0; i < 3; i++)
          {
            if(local_handle_msg_.motorError[i] == 128 || local_handle_msg_.motorError[i] == 129 )
               hand_temperature_counter_[i] += 1;
          }
        }
        else
        {

        RobotStatusCodes::StatusLevel hand_status_level;
        RobotStatusCodes::StatusCode hand_status_code;
        flor_ocs_msgs::OCSRobotStatus temp_status;

        hand_status_level = RobotStatusCodes::OK;
        hand_status_code = RobotStatusCodes::NO_ERROR;


        // Finger 1 status message
        if(hand_temperature_counter_[0] > 1)
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_1_TEMPERATURE;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_1_TEMPERATURE;
            hand_status_level = RobotStatusCodes::WARNING;
        }
        else if(local_handle_msg_.responseHistory[1] <= 75)
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_1_NO_DATA;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_1_NO_DATA;

            if(local_handle_msg_.responseHistory[1] == 0 && local_handle_msg_.responseHistory[2] == 0)
                hand_status_level = RobotStatusCodes::ERROR;
            else
                hand_status_level = RobotStatusCodes::WARNING;
        }
        else
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_1_OK;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_1_OK;
        }

        // Publish finger 2 status if status has changed
        temp_status.status = RobotStatusCodes::status(hand_status_code, hand_status_level);
        if(temp_status.status != hand_status_[0].status)
        {
            hand_status_[0].status = temp_status.status;
            hand_status_[0].stamp = ros::Time::now();
            irobot_status_pub_.publish(hand_status_[0]);
        }

        usleep(1000*10);

        hand_status_level = RobotStatusCodes::OK;
        hand_status_code = RobotStatusCodes::NO_ERROR;

        // Finger 2 status message
        if(hand_temperature_counter_[1] > 1)
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_2_TEMPERATURE;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_2_TEMPERATURE;
            hand_status_level = RobotStatusCodes::WARNING;
        }
        else if(local_handle_msg_.responseHistory[3] <= 75)
        {
            //ROS_INFO("finger 2 not responding");
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_2_NO_DATA;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_2_NO_DATA;

            if(local_handle_msg_.responseHistory[3] == 0 && local_handle_msg_.responseHistory[4] == 0)
                hand_status_level = RobotStatusCodes::ERROR;
            else
                hand_status_level = RobotStatusCodes::WARNING;
        }
        else
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_2_OK;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_2_OK;
        }

        // Publish finger 2 status if status has changed
        temp_status.status = RobotStatusCodes::status(hand_status_code, hand_status_level);
        if(temp_status.status != hand_status_[1].status)
        {
            hand_status_[1].status = temp_status.status;
            hand_status_[1].stamp = ros::Time::now();
            irobot_status_pub_.publish(hand_status_[1]);
        }

        hand_status_level = RobotStatusCodes::OK;
        hand_status_code = RobotStatusCodes::NO_ERROR;

        // Finger 3 status message
        if(hand_temperature_counter_[2] > 1)
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_3_TEMPERATURE;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_3_TEMPERATURE;
            hand_status_level = RobotStatusCodes::WARNING;
        }
        else if(local_handle_msg_.responseHistory[5] <= 75)
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_3_NO_DATA;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_3_NO_DATA;

            if(local_handle_msg_.responseHistory[5] == 0 && local_handle_msg_.responseHistory[6] == 0)
                hand_status_level = RobotStatusCodes::ERROR;
            else
                hand_status_level = RobotStatusCodes::WARNING;
        }
        else
        {
            if(hand_id_ > 0)
                hand_status_code = RobotStatusCodes::RIGHT_FINGER_3_OK;
            else
                hand_status_code = RobotStatusCodes::LEFT_FINGER_3_OK;
        }

        // Publish finger 3 status if status has changed
        temp_status.status = RobotStatusCodes::status(hand_status_code, hand_status_level);
        if(temp_status.status != hand_status_[2].status)
        {
            hand_status_[2].status = temp_status.status;
            hand_status_[2].stamp = ros::Time::now();
            irobot_status_pub_.publish(hand_status_[2]);
        }

        publish_counter_ = 0;
        hand_temperature_counter_[0] = 0;
        hand_temperature_counter_[1] = 0;
        hand_temperature_counter_[2] = 0;
        }
    }

    void VigirIRobotGraspController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
    {
        tf::Vector3 hand_com = tf::Vector3(0.0,0.0,0.0);
        hand_com.setY(hand_id_ < 1 ? 0.18: -0.18);
        float      hand_mass = 1.56;
        template_com = hand_T_template * template_com;
        hand_com     *= hand_mass;
        template_com *= template_mass;
        hand_com     += template_com;

        this->hand_mass_msg_.hand  = hand_id_ < 1 ? 0: 1;  //TO KEEP CONSISTENCY WITH BDI LEFT=0/RIGHT=1
        this->hand_mass_msg_.mass = hand_mass + template_mass;
        if(this->hand_mass_msg_.mass >= hand_mass && this->hand_mass_msg_.mass <= 10.0) //Mass sanity check, we don't consider masses over 10 Kg and mass shouldn't be less than actual hand mass
            hand_com            /= this->hand_mass_msg_.mass;
        else{   //If sanity check failed, reset the mass and CoM to the corresponding hand.
            ROS_WARN("Mass sanity check failed, resetting to actual hand mass (hand mass = %f + template mass= %f).",hand_mass,template_mass);
            this->hand_mass_msg_.mass = hand_mass;
            hand_com.setX(0.0);
            hand_com.setY(hand_id_ < 1 ? 0.18: -0.18);
            hand_com.setZ(0.0);
        }
        com_.header.frame_id = "/"+this->hand_name_;

        if (fabs(hand_com.getX()) < 1.0 && fabs(hand_com.getY()) < 1.0 && fabs(hand_com.getZ()) < 1.0) //CoM sanity check, we don't consider positions over 1 meter.
        {
            com_.pose.position.x = hand_com.getX();
            com_.pose.position.y = hand_com.getY();
            com_.pose.position.z = hand_com.getZ();
        }
        else{//CoM sanity check failed, reset the CoM to the corresponding hand.
            ROS_WARN("CoM sanitiy check failed, resetting to actual hand CoM.");
            this->hand_mass_msg_.mass = hand_mass;
            com_.pose.position.x = 0.0;
            com_.pose.position.y = hand_id_ < 1 ? 0.18: -0.18;
            com_.pose.position.z = 0.0;
        }
        this->hand_mass_msg_.com = com_;
    }

    void VigirIRobotGraspController::publishHandStates(bool update_joint_commands)
    {
       // call as needed to publish the updated joint commands
       if (update_joint_commands)
       {
           for(int i = 0; i < 3; i++)
                joint_commands_.value[i] += zero_joint_values_[i];
           joint_commands_pub_.publish(joint_commands_);
       }
       tactile_feedback_msg_.header.stamp = ros::Time::now();
       irobot_states_pub_.publish(tactile_feedback_msg_);
       // @todo - for Physical hand, should we load sensor data into simulation structures and publish to mimic simulation?
       //   who in OCS/onboard is using that form of the data?

    }

    void VigirIRobotGraspController::setInitialFingerPoses(const uint8_t& grasp_type)
    {
        zero_velocity_counter_ = 0;
        if (grasp_type < this->initial_finger_poses_.size())
            this->finger_poses_  = this->initial_finger_poses_.at(grasp_type);
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
    }

    void VigirIRobotGraspController::setFinalFingerPoses(const uint8_t& grasp_type)
    {
        zero_velocity_counter_ = 0;
        if (grasp_type < this->final_finger_poses_.size())
        {
            this->finger_poses_  = this->final_finger_poses_.at(grasp_type);

            float max ;
            max = std::max(this->finger_poses_.fingers[0],this->finger_poses_.fingers[1]);
            max = std::max(max,this->finger_poses_.fingers[2]);
            //min = std::min_element(this->finger_poses_.fingers[0],this->finger_poses_.fingers[2]);
            this->finger_close_scale_ = (7000 - max) ;
            ROS_INFO("Minimum finger value: %f ; finger scale value: %d", max, this->finger_close_scale_);
            if(this->finger_close_scale_ < 0)
                this->finger_close_scale_ = 100;
        }
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
        for(unsigned int i = 0; i < this->joint_names_.size(); ++i)
        {
          // Graspit outputs the fingers in different order, and these were copied into .grasp library
          // We need to swap f0 and f2, which this code does
            ROS_INFO("Finger Joint %d = %f",i,this->finger_poses_.fingers[i]);
        }
    }

    void VigirIRobotGraspController::setInitialJointToCurrentPositions()
    {

        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
            if (i == 4)
                real_initial_finger_poses_.fingers[i] = this->local_handle_msg_.fingerSpread; //stores real joint positions
            else
                real_initial_finger_poses_.fingers[i] = this->local_handle_msg_.motorHallEncoder[i]; //stores real joint positions
        }
    }

    void VigirIRobotGraspController::setFingerPosesToID(const uint16_t& requested_release_id, const uint16_t& grasp_template_id)
    {// Assumes data is locked by calling thread
        for (std::vector <VigirIRobotGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
        {
            if((*it).grasp_id == requested_release_id)
            {
                this->grasp_id_       = (*it).grasp_id;
                this->template_id_    = grasp_template_id;
                this->template_type_  = (*it).template_type;
                this->finger_poses_   = (*it).finger_poses;   // assumes defined array fully copied

                for(unsigned int i = 0; i < 5; ++i)
                {
                    if (i<4)
                     finger_poses_.fingers[i] = finger_poses_.fingers[i] * rad_to_pos_conversion;
                    else
                     finger_poses_.fingers[i] = finger_poses_.fingers[i] * spread_conversion;
                  // Graspit outputs the fingers in different order, and these were copied into .grasp library
                  // We need to swap f0 and f2, which this code does
                    ROS_INFO("Finger Joint %d = %f",i,this->finger_poses_.fingers[i]);
                }

                // This is for determining the scaling factor for the feedforward effort for the iRobot hand.
                // We do not want a constant scale, since some grasps may be more closed than others.
                float max ;
                max = std::max(this->finger_poses_.fingers[0],this->finger_poses_.fingers[1]);
                max = std::max(max,this->finger_poses_.fingers[2]);
                //min = std::min_element(this->finger_poses_.fingers[0],this->finger_poses_.fingers[2]);
                this->finger_close_scale_ = (7000 - max) ;
                ROS_INFO("Minimum finger value: %f ; finger scale value: %d", max, this->finger_close_scale_);
                if(this->finger_close_scale_ < 0)
                    this->finger_close_scale_ = 100;

                // Store zero values for finger tactile data in order to calibrate data
                int fail_count = 0;
                for(int i = 0; i < 3; i++)
                {
                    for(int j = 0; j < 12; j++)
                    {
                        if(local_handle_msg_.fingerTactile[i].proximal[j] < -500 && fail_count < 10)
                        {
                            ros::spinOnce();
                            fail_count++;
                            j--;
                        }
                        else
                        {
                            tactile_zeros_[i].proximal[j] = local_handle_msg_.fingerTactile[i].proximal[j];
                            fail_count = 0;
                        }
                    }
                    for(int j = 0; j < 10; j++)
                    {
                        if(local_handle_msg_.fingerTactile[i].distal[j] < -500 && fail_count < 10)
                        {
                            ros::spinOnce();
                            fail_count++;
                            j--;
                        }
                        else
                        {
                            tactile_zeros_[i].distal[j] = local_handle_msg_.fingerTactile[i].distal[j];
                            fail_count = 0;
                        }
                    }
                }
                for(int i = 0; i < 48; i++)
                {
                    if(local_handle_msg_.palmTactile[i] < -500)
                        ros::spinOnce();
                    else
                        palm_tactile_zeros_[i] = local_handle_msg_.palmTactile[i];
                }
            }
        }

        if ( (this->grasp_id_ != requested_release_id))
        {
            ROS_WARN("%s: Release grasp selection id: %d not found.",
                     hand_name_.c_str(), requested_release_id);
        }

    }

    void VigirIRobotGraspController::updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type)
    {
        for (std::vector <VigirIRobotGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
        {
           if((*it).grasp_id == requested_grasp_id && (*it).template_type == requested_template_type)
           {
               boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
               this->grasp_id_            = (*it).grasp_id;
               this->template_type_       = (*it).template_type;
               this->template_id_         = requested_template_id;
               this->finger_poses_        = (*it).finger_poses;   // assumes defined array fully copied
               this->final_wrist_pose_    = (*it).final_wrist_pose;
               this->pregrasp_wrist_pose_ = (*it).pregrasp_wrist_pose;
               this->grasp_type_          = (*it).grasp_type;



               // Force a new round of template match to be sure and get wrist pose
               this->template_updated_    = false;
               this->grasp_updated_       = true;

               for(unsigned int i = 0; i < 5; ++i)
               {
                   if (i<4)
                    finger_poses_.fingers[i] = finger_poses_.fingers[i] * rad_to_pos_conversion;
                   else
                    finger_poses_.fingers[i] = finger_poses_.fingers[i] * spread_conversion;
                 // Graspit outputs the fingers in different order, and these were copied into .grasp library
                 // We need to swap f0 and f2, which this code does
                   ROS_INFO("Finger Joint %d = %f",i,this->finger_poses_.fingers[i]);
               }

               // This is for determining the scaling factor for the feedforward effort for the iRobot hand.
               // We do not want a constant scale, since some grasps may be more closed than others.
               float max ;
               max = std::max(this->finger_poses_.fingers[0],this->finger_poses_.fingers[1]);
               max = std::max(max,this->finger_poses_.fingers[2]);
               //min = std::min_element(this->finger_poses_.fingers[0],this->finger_poses_.fingers[2]);
               this->finger_close_scale_ = (7000 - max) ;
               ROS_INFO("Minimum finger value: %f ; finger scale value: %d", max, this->finger_close_scale_);
               if(this->finger_close_scale_ < 0)
                   this->finger_close_scale_ = 100;

               // Store zero values for finger tactile data in order to calibrate data
               for(int i = 0; i < 3; i++)
               {
                   for(int j = 0; j < 12; j++)
                   {
                       if(local_handle_msg_.fingerTactile[i].proximal[j] < -500)
                           ros::spinOnce();
                       else
                           tactile_zeros_[i].proximal[j] = local_handle_msg_.fingerTactile[i].proximal[j];
                   }
                   for(int j = 0; j < 10; j++)
                   {
                       if(local_handle_msg_.fingerTactile[i].distal[j] < -500)
                           ros::spinOnce();
                       else
                           tactile_zeros_[i].distal[j] = local_handle_msg_.fingerTactile[i].distal[j];
                   }
               }
               for(int i = 0; i < 48; i++)
               {
                   if(local_handle_msg_.palmTactile[i] < -500)
                       ros::spinOnce();
                   else
                       palm_tactile_zeros_[i] = local_handle_msg_.palmTactile[i];
               }
           }

        }

        if (-1 == this->grasp_id_ ) ROS_WARN("Failed to match grasp ID - set to -1");

    }



    void VigirIRobotGraspController::setHandEfforts(const double& grasp_effort, const double finger_effort[])
    {
        //Effort settings in MANUAL mode
        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
            this->joint_commands_.type[i] = 1; // Set to velocity control type
            this->joint_commands_.valid[i] = true; // Enable control of the hand

           switch(i){
           case 0: this->joint_commands_.value[i] = 0.00001 + grasp_effort*0.03+finger_effort[1]*0.03; //Left
               break;
           case 1: this->joint_commands_.value[i] = 0.00001 + grasp_effort*0.03+finger_effort[2]*0.03; //Right
               break;
           case 2: this->joint_commands_.value[i] = 0.00001 + grasp_effort*0.03+finger_effort[0]*0.03; //Thumb
               break;
           case 4: this->joint_commands_.value[i] = 0.00001 + grasp_effort*0.03+finger_effort[3]*0.03; //Spread
               break;
           default: this->joint_commands_.value[i] = 0.00001 + grasp_effort*0.02;
               break;
           }

        }
    }

    void VigirIRobotGraspController::setAttachingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data)
    {
    }

    void VigirIRobotGraspController::setDetachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data  )
    {
    }

    void VigirIRobotGraspController::setStitchingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data)
    {
    }

    void VigirIRobotGraspController::setHandNoneData( )
    {
        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
            this->joint_commands_.type[i] = 2; // Set to position control type

            current_finger_poses_.fingers[i] = this->final_finger_poses_.at(GRASP_NONE).fingers[i];
            this->joint_commands_.value[i] = current_finger_poses_.fingers[i];
            this->joint_commands_.valid[i]   = true;
        }
    }

    void VigirIRobotGraspController::setHandApproachingData(const double& grasp_fraction)
    {
        double one_minus_fraction = 1.0 - grasp_fraction;

        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
           this->joint_commands_.type[i] = 2; // Set to position control type

           current_finger_poses_.fingers[i] = real_initial_finger_poses_.fingers[i] * one_minus_fraction +  //moving from the actual joint states avoids jumping from a full close to a full open, or between any two different grasps.
                                             initial_finger_poses_.at(this->grasp_type_).fingers[i] * grasp_fraction;
           this->joint_commands_.value[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.valid[i]   = true;
        }



    }
    void VigirIRobotGraspController::setHandSurroundingData( )
    {
        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
           this->joint_commands_.type[i] = 2; // Set to position control type

           current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i]; // waste to recalc, but otherwise must keep track of transition
           this->joint_commands_.value[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.valid[i]   = true;
        }
    }

    void VigirIRobotGraspController::setHandGraspingData(const double& grasp_fraction, const int8_t finger_effort[])
    {

        double one_minus_fraction = 1.0 - grasp_fraction;
        double grasp_fraction_f1  = grasp_fraction * ratio_f1_f2;
        if(grasp_fraction_f1 > 1.0)
           grasp_fraction_f1 = 1.0;

        // Check to see if the command has changed, if not, increment counter

          for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
          {
            if(joint_commands_.type[i] == 2)
                old_joint_commands_.value[i] = joint_commands_.value[i];
            this->joint_commands_.type[i] = 2; // Set to position control type

            switch(i){
            case 0: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                        this->finger_poses_.fingers[i] * grasp_fraction + finger_effort[1] * 50;  //Left
                break;
            case 1: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                        this->finger_poses_.fingers[i] * grasp_fraction + finger_effort[2] * 50;  //Right
                break;
            case 2: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                        this->finger_poses_.fingers[i] * grasp_fraction + finger_effort[0] * 50;  //Thumb
                break;
            case 4: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                        this->finger_poses_.fingers[i] * grasp_fraction + finger_effort[3] * 15;  //Spread
                break;
            default: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                        this->finger_poses_.fingers[i] * grasp_fraction;
                break;
            }

           //std::cout << "        f" << i << "  position=" << current_finger_poses_.f0[i] << " = " << initial_finger_poses_.at(this->grasp_type_).f0[i] << " + " << this->finger_poses_.f0[i]<< std::endl;
           this->joint_commands_.value[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.valid[i] = true; // Set to position control type
           //this->joint_commands_.effort[i]   = grasp_effort*0.001; // @TODO - fix the scaling on this
          }

        ///*
          bool finger_command_changed = false;
          for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
          {
              if(old_joint_commands_.value[i] != joint_commands_.value[i])
                  finger_command_changed = true;
          }
          if(!finger_command_changed)
          {
              if(this->zero_velocity_counter_ < 75)
                  this->zero_velocity_counter_++;
          }
          else
              zero_velocity_counter_ = 0;

          if(zero_velocity_counter_ >= 75 && !finger_command_changed)
          {
            for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
            {
                //old_joint_commands_.value[i] = 0;
                this->joint_commands_.type[i] = 1; // Set to velocity control type
                this->joint_commands_.value[i] = 0;
                this->joint_commands_.valid[i] = true;
            }
         }
        ///*/
    }

    void VigirIRobotGraspController::setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])
    {

        // Check to see if the command has changed, if not, increment counter




        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
          if(joint_commands_.type[i] == 2)
            old_joint_commands_.value[i] = joint_commands_.value[i];
          this->joint_commands_.type[i] = 2; // Set to position control type

          current_finger_poses_.fingers[i] = this->finger_poses_.fingers[i]; // waste to recalc, but otherwise must keep track of transition
          this->joint_commands_.value[i] = current_finger_poses_.fingers[i];

          // HACKED THUMB EFFORT FOR DRILLING TASK TO BE TRIGGER FINGER EFFORT

          switch(i){
          case 0: this->joint_commands_.value[i] += finger_effort[1] * 50; //Left
              break;
          case 1: this->joint_commands_.value[i] += finger_effort[2] * 50; //Right
              break;
          case 2: this->joint_commands_.value[i] += finger_effort[0] * 50; //Thumb
              break;
          case 4: this->joint_commands_.value[i] += finger_effort[3] * 15; //Spread
              break;
          default:
              break;
          }

          if (i <3)
              this->joint_commands_.value[i] += grasp_effort * this->finger_close_scale_ /100;
          this->joint_commands_.valid[i] = true; // Set to position control type
        }


        bool finger_command_changed = false;
        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
            if(old_joint_commands_.value[i] != joint_commands_.value[i])
                finger_command_changed = true;
        }
        if(!finger_command_changed)
        {
            if(this->zero_velocity_counter_ < 75)
                this->zero_velocity_counter_++;
        }
        else
            zero_velocity_counter_ = 0;


        if(zero_velocity_counter_ >= 75 && !finger_command_changed)
        {
            for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
            {
                //old_joint_commands_.value[i] = 0;
                this->joint_commands_.type[i] = 1; // Set to velocity control type
                this->joint_commands_.value[i] = 0;
                this->joint_commands_.valid[i] = true;
            }
        }


    }

    void VigirIRobotGraspController::setHandOpeningData(const double& grasp_fraction)
    {

        double one_minus_fraction = 1.0 - grasp_fraction;
        for(unsigned int i = 0; i < NUM_IROBOT_FINGER_JOINTS; ++i)
        {
            this->joint_commands_.type[i] = 2; // Set to position control type

            current_finger_poses_.fingers[i] = real_initial_finger_poses_.fingers[i] * one_minus_fraction +
                                                initial_finger_poses_.at(this->grasp_type_).fingers[i] * grasp_fraction;
            this->joint_commands_.value[i] = current_finger_poses_.fingers[i];
            this->joint_commands_.valid[i]   = true;
        }


    }

    /////// ---------------------------------- Hardware callbacks ---------------------------------------
    // Hand specific callbacks used for processing hardware data
    void VigirIRobotGraspController::handleSensorsCallback(const handle_msgs::HandleSensors::ConstPtr &hs_msg)
    {
      boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

      // Update Joint State information
      // use calibrated joint values to zero feedback
      last_joint_state_msg_ = hs_msg;


      // This call triggers processing of data and handling grasp calculations with worker thread loop
      this->updatedSensorData();
      //ROS_INFO("sensor data received: %d", updated_sensor_data_);
    }

    void VigirIRobotGraspController::calibrateCallback(const sensor_msgs::JointStateConstPtr &cal_msg)
    {
      boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

      // Update Joint State information
      for(int i = 0; i < 3; i++)
      {
          zero_joint_values_[i] = cal_msg->position[i];
      }

    }

    /////// ---------------------------------- End Hardware callbacks ---------------------------------------


    // Hand specific messages for handling data
    bool FlorGraspSpecificationSorter(VigirIRobotGraspSpecification const& a, VigirIRobotGraspSpecification const& b)
    {
        if (a.grasp_id < b.grasp_id) return true;
        return false;
    }

    void VigirIRobotGraspController::loadIRobotGraspDatabase(std::string& file_name)
    {
        /// @TODO - need to add expection protection for reading data out of range

        ROS_INFO("Setup default joint names for the %s hand", hand_name_.c_str());

        // Read the files to load all template and grasping data

        // TODO - need to make this a paramter that we can set by calling from plugin
        std::ifstream file ( file_name.c_str() );
        std::string grasp_line;
        VigirIRobotGraspSpecification grasp_spec;
        std::string tmp_hand_name;
        int8_t      tmp_hand_id;

        potential_grasps_.clear();
        potential_grasps_.push_back(VigirIRobotGraspSpecification()); // default 0 grasp

        this->joint_names_.clear(); // reset the vector

        // must match those inside of the /irobot_hands/?_hand/joint_states/[right_/left_]+
        if(hand_id_ < 0)
        {
            tmp_hand_name = "left_";
        }
        else
        {
            tmp_hand_name = "right_";
        }
        this->joint_names_.push_back(tmp_hand_name+"f0_j0");
        this->joint_names_.push_back(tmp_hand_name+"f1_j0");
        this->joint_names_.push_back(tmp_hand_name+"f2_j0");
        this->joint_names_.push_back(tmp_hand_name+"f2_j1");
        this->joint_names_.push_back(tmp_hand_name+"sprd");

        if (NUM_IROBOT_FINGER_JOINTS != this->joint_names_.size())
        {   // should throw exception here and abort plugin loading
            ROS_ERROR("Invalid joint names - don't match our vectors %d vs. %d", uint8_t(this->joint_names_.size()), NUM_IROBOT_FINGER_JOINTS);
        }


        if (file.is_open())
        {
          while ( file.good() )
          {
            getline ( file, grasp_line);
            if(grasp_line.find('#')==0 || grasp_line.size() <= 0){
                continue;
            }
            std::stringstream values(grasp_line);
            std::string value;
            std::stringstream trimmer;
            std::getline( values, value, ',' );
            grasp_spec.grasp_id = atoi(value.c_str()); //grasp id
            if (0 == grasp_spec.grasp_id)
            {
                ROS_INFO(" Skipping grasp specification 0 for %s given %s controller", tmp_hand_name.c_str(), hand_name_.c_str());
                continue;
            }

            getline( values, value, ',' ); //template type
            grasp_spec.template_type = atoi(value.c_str());

            getline( values, value, ',' );  //hand
            trimmer << value;
            value.clear();
            trimmer >> value; //removes white spaces
            if(value.compare("left") == 0)
            {
                tmp_hand_name = "l_hand";
                tmp_hand_id = -1;
            }
            else
            {
              tmp_hand_name = "r_hand";
              tmp_hand_id = 1;
            }

            if (tmp_hand_id != hand_id_)
            {
                ROS_INFO(" Skipping grasp specification for %s given %s controller", tmp_hand_name.c_str(), hand_name_.c_str());
                continue;
            }

            std::string tmp_grasp_type;
            getline( values, tmp_grasp_type, ',' ); //grasp_type
            trimmer.clear();
            trimmer << tmp_grasp_type;
            tmp_grasp_type.clear();
            trimmer >> tmp_grasp_type; //removes white spaces
            grasp_spec.grasp_type = GRASP_CYLINDRICAL;
            if ("spherical" == tmp_grasp_type)
            {
                grasp_spec.grasp_type = GRASP_SPHERICAL;
            } else if ("prismatic" == tmp_grasp_type) {
                grasp_spec.grasp_type = GRASP_PRISMATIC;
            } else if ("cylindrical" == tmp_grasp_type) {
                grasp_spec.grasp_type = GRASP_CYLINDRICAL;
            } else {
                ROS_WARN(" Unknown grasp type <%s> assuming cylindrical", tmp_grasp_type.c_str());
            }

            getline( values, value, ',' ); // "finger poses:,"
            assert(NUM_IROBOT_FINGER_JOINTS == this->joint_names_.size());
            // TODO: fix irobot joint values

            for(unsigned int i = 0; i < this->joint_names_.size(); ++i)
            {
              getline( values, value, ',' );

              // Graspit outputs the fingers in different order, and these were copied into .grasp library
              // We need to swap f0 and f2, which this code does
              ROS_INFO("Finger Joint %d = %f",i,atof(value.c_str()));
              grasp_spec.finger_poses.fingers[i]= atof(value.c_str()); //joints from the pinky
            }

            float ftmp;

            getline( values, value, ',' ); // "final pose:,"
            for(unsigned int i = 0; i < 7; ++i)
            {
              getline( values, value, ',' );
              ftmp = atof(value.c_str()); //final pose values
              if (ftmp > 2.0)
              {
                  ROS_ERROR("Found pose value > 2.0 in final wrist - Is this file using millimeters again - should be meters!");
              }
              switch(i)
              {
                  case 0: grasp_spec.final_wrist_pose.position.x    = ftmp; break;
                  case 1: grasp_spec.final_wrist_pose.position.y    = ftmp; break;
                  case 2: grasp_spec.final_wrist_pose.position.z    = ftmp; break;
                  case 3: grasp_spec.final_wrist_pose.orientation.w = ftmp; break;
                  case 4: grasp_spec.final_wrist_pose.orientation.x = ftmp; break;
                  case 5: grasp_spec.final_wrist_pose.orientation.y = ftmp; break;
                  case 6: grasp_spec.final_wrist_pose.orientation.z = ftmp; break;
              }
            }
            //Static transformation to /r_hand
            staticTransform(grasp_spec.final_wrist_pose);

            getline( values, value, ',' ); // "pre-grasp pose:,"
            for(unsigned int i = 0; i < 7; ++i)
            {
              getline( values, value, ',' );
              ftmp = atof(value.c_str()); //pre-grasp values
              if (ftmp > 2.0)
              {
                  ROS_ERROR("Found pose value > 2.0 in pre-grasp wrist - Is this file using millimeters again - should be meters!");
              }
              switch(i)
              {
                  case 0: grasp_spec.pregrasp_wrist_pose.position.x    = ftmp; break;
                  case 1: grasp_spec.pregrasp_wrist_pose.position.y    = ftmp; break;
                  case 2: grasp_spec.pregrasp_wrist_pose.position.z    = ftmp; break;
                  case 3: grasp_spec.pregrasp_wrist_pose.orientation.w = ftmp; break;
                  case 4: grasp_spec.pregrasp_wrist_pose.orientation.x = ftmp; break;
                  case 5: grasp_spec.pregrasp_wrist_pose.orientation.y = ftmp; break;
                  case 6: grasp_spec.pregrasp_wrist_pose.orientation.z = ftmp; break;
              }
            }
            //Static transformation to /r_hand
            staticTransform(grasp_spec.pregrasp_wrist_pose);

            ROS_INFO("Loading Grasp from file: Id: %d, Template type: %d, Hand name: %s, Hand id: %d, Grasp type: %s",
                     grasp_spec.grasp_id,grasp_spec.template_type,tmp_hand_name.c_str(),
                     tmp_hand_id,tmp_grasp_type.c_str());
            ROS_INFO("      pre-grasp   target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
                     grasp_spec.pregrasp_wrist_pose.position.x,    grasp_spec.pregrasp_wrist_pose.position.z,grasp_spec.pregrasp_wrist_pose.position.z,
                     grasp_spec.pregrasp_wrist_pose.orientation.w, grasp_spec.pregrasp_wrist_pose.orientation.x, grasp_spec.pregrasp_wrist_pose.orientation.y, grasp_spec.pregrasp_wrist_pose.orientation.z);
            ROS_INFO("      final grasp target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
                     grasp_spec.final_wrist_pose.position.x,    grasp_spec.final_wrist_pose.position.z,grasp_spec.final_wrist_pose.position.z,
                     grasp_spec.final_wrist_pose.orientation.w, grasp_spec.final_wrist_pose.orientation.x, grasp_spec.final_wrist_pose.orientation.y, grasp_spec.final_wrist_pose.orientation.z);
            potential_grasps_.push_back(grasp_spec);
          }
          file.close();
        }else
        {
          ROS_ERROR("Error loading grasp library from : %s",file_name.c_str());
        }

        // Let's sort the grasp id's into numerical order to allow fast location
        ROS_INFO(" Sort grasp identifiers ...");

        std::sort(potential_grasps_.begin(), potential_grasps_.end(), FlorGraspSpecificationSorter);

        // Convert radians to motor input values
        const float  spread_conversion = 760/1.57;
        const float  rad_to_pos_conversion = 3500/1.57; // 3500/1.57

        // probably should dump to screen to verify that data is still correct.
        ROS_INFO(" Setup initial finger poses for grasp types ...");
        initial_finger_poses_.resize(NUM_GRASP_TYPES);
        final_finger_poses_.resize(NUM_GRASP_TYPES);

        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[0]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[1]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[2]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[4]=0.0;

        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[0]=1.3 * rad_to_pos_conversion ;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[1]=1.3 * rad_to_pos_conversion;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[2]=1.3 * rad_to_pos_conversion;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[4]=0.0;

        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[0]=1.25 * rad_to_pos_conversion ;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[1]=1.25 * rad_to_pos_conversion ;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[2]=1.25 * rad_to_pos_conversion ;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[4]=0.7 * spread_conversion;

        initial_finger_poses_.at(GRASP_NONE).fingers[0]=0;
        initial_finger_poses_.at(GRASP_NONE).fingers[1]=0;
        initial_finger_poses_.at(GRASP_NONE).fingers[2]=0;
        initial_finger_poses_.at(GRASP_NONE).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_NONE).fingers[4]=0.0;


        // TODO - define terminal finger positions with fully closed manual grasp

        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[0]=2.5 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[1]=2.5 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[2]=2.5 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[4]=0.0;

        final_finger_poses_.at(GRASP_PRISMATIC).fingers[0]=2.250 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[1]=2.250 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[2]=2.250 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[4]=0.0;

        final_finger_poses_.at(GRASP_SPHERICAL).fingers[0]=1.75 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[1]=1.75 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[2]=1.75 * rad_to_pos_conversion;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[4]=0.7 * spread_conversion;

        final_finger_poses_.at(GRASP_NONE).fingers[0]=2.5 * rad_to_pos_conversion;;
        final_finger_poses_.at(GRASP_NONE).fingers[1]=2.5 * rad_to_pos_conversion;;
        final_finger_poses_.at(GRASP_NONE).fingers[2]=2.5 * rad_to_pos_conversion;;
        final_finger_poses_.at(GRASP_NONE).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_NONE).fingers[4]=0.0;


        // Initialize states and control values
        active_state_.grasp_state.data = (TEMPLATE_GRASP_MODE << 4) + GRASP_STATE_NONE; // Nothing is specified until we get a grasp command AND template pose
        grasp_id_       = -1 ;
        template_id_    = -1 ;
        template_type_  = -1 ;
        grasp_type_     = GRASP_CYLINDRICAL;
        start_grasp_flag = false;
        grasp_period     = 1.0;    // in seconds
        grasp_gain       = 10.0;
        finger_open_threshold  = 0.99;
        finger_close_threshold = 0.95;
        grasp_status_.status  = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_code_    = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_severity_= RobotStatusCodes::OK;
        ROS_INFO(" Done initializing grasp library! ");

    }



    GraspQuality VigirIRobotGraspController::processHandTactileData()
    {
        bool thumb      = false,
             index      = false,
             middle     = false,
             //pinky      = false,
             inner_palm = false;

        unsigned int count      = 0;
        uint16_t average_filter = 0.0;
        float temp_tactile = 0;
        // Smoothing of tactile data
        for(int i = 0; i < 3; i++)
        {
          for(unsigned int j = 0; j < 12; j++)
          {
              // If there is no tactile data from fingers, set tactile data to -1.0
              if(local_handle_msg_.responseHistory[1+i*2] == 0)
              {
                  this->tactile_timeout_counter_[i*2] += 1;
              }
              else
                  tactile_timeout_counter_[i*2] = 0;

              if(this->tactile_timeout_counter_[i*2] > 50)
              {
                  this->filtered_tactile_msg_[i].proximal[j] = -1.0;
                  tactile_timeout_counter_[i*2] = 50;
              }
              else if(local_handle_msg_.responses[1 + i*2])
              {
                  // Only add in positive tactile data
                temp_tactile = this->local_handle_msg_.fingerTactile[i].proximal[j]-tactile_zeros_[i].proximal[j];
                if(temp_tactile > 0)
                  this->filtered_tactile_msg_[i].proximal[j] = this->filtered_tactile_msg_[i].proximal[j]*0.9 + temp_tactile *0.1;
                else
                  this->filtered_tactile_msg_[i].proximal[j] = this->filtered_tactile_msg_[i].proximal[j]*0.9;
              }
          }
          for(unsigned int j = 0; j < 10; j++)
          {
              // If there is no tactile data from fingers, set tactile data to -1.0
              if(local_handle_msg_.responseHistory[2 + i*2] == 0)
                  this->tactile_timeout_counter_[1 + 2*i] += 1;
              else
                  tactile_timeout_counter_[1 +i*2] = 0;

              if(tactile_timeout_counter_[1 + 2 * i] > 50)
              {
                  this->filtered_tactile_msg_[i].distal[j] = -1.0;
                  tactile_timeout_counter_[i*2] = 50;
              }
              else if(local_handle_msg_.responses[2 + i*2])
              {
                  // Only add in positive tactile data
                temp_tactile = this->local_handle_msg_.fingerTactile[i].distal[j]-tactile_zeros_[i].distal[j];
                if(temp_tactile > 0)
                  this->filtered_tactile_msg_[i].distal[j] = this->filtered_tactile_msg_[i].distal[j]*0.9 + temp_tactile *0.1;
                else
                  this->filtered_tactile_msg_[i].distal[j] = this->filtered_tactile_msg_[i].distal[j]*0.9;
              }
          }
        }
        for(unsigned int i = 0; i < 48; i++)
        {
            // Only add in positive tactile data
            temp_tactile = this->local_handle_msg_.palmTactile[i]-palm_tactile_zeros_[i];
            if(temp_tactile > 0)
                this->filtered_palm_tactile_msg_[i]=this->filtered_palm_tactile_msg_[i]*0.9+(temp_tactile)*0.1;
            else
                this->filtered_palm_tactile_msg_[i]=this->filtered_palm_tactile_msg_[i]*0.9;
        }

        // Processing of finger tactile information
        for (unsigned int i = 0; i < 3; i++)
        {
            count      = 0;
            average_filter = 0.0;

            if(local_handle_msg_.responses[i*2+1]) // Do not process data if it was not received from handle_node
            {
            // Processing of Proximal Phalange tactile data
            for(unsigned int j=0; j<12; j++)
            {
                if(filtered_tactile_msg_[i].proximal[j] > 0.20)
                {
                    switch(i)
                    {
                    case 0:
                        index = true;
                        break;
                    case 1:
                        middle = true;
                        break;
                    case 2:
                        thumb = true;
                        break;
                    }
                    average_filter+=filtered_tactile_msg_[i].proximal[j];
                    count++;
                }
            }
            //int k = i * 2;
            if(count>0)
                tactile_feedback_msg_.velocity[i] = (average_filter/count)/300.0;
                //this->filtered_tactile_data_[i].proximal[0] =(average_filter/count)/300.0;
            else
                tactile_feedback_msg_.velocity[i] = 0.0;
            }

            count      = 0;
            average_filter = 0.0;

            // Processing of Distal Phalange tactile data
            if(local_handle_msg_.responses[i*2+2])// Do not process data if it was not received from handle_node
            {
            for(unsigned int j=0; j<10; j++)
            {
                if(filtered_tactile_msg_[i].distal[j] > 0.20)
                {
                    switch(i)
                    {
                    case 0:
                        index = true;
                        break;
                    case 1:
                        middle = true;
                        break;
                    case 2:
                        thumb = true;
                        break;
                    }

                    average_filter+=filtered_tactile_msg_[i].distal[j];
                    count++;
                }
            }
            //int k = i*2 + 1;
            if(count>0)
            {
                tactile_feedback_msg_.velocity[i+5] = (average_filter/float(count)) / 300 ;
            }
            else
                tactile_feedback_msg_.velocity[i+5] = 0.0;
            }
        }

        //Index Base (Palm tactile)
        average_filter = 0;
        count          = 0;
        for(unsigned int i=0; i<15; ++i)
        {
            if(filtered_palm_tactile_msg_[i]>0)
            {
                average_filter+=filtered_palm_tactile_msg_[i];
                count++;
            }
        }
        if(count>0)
            //tactile_feedback_msg_.velocity[3] = (average_filter/count)/300.0;
            this->tactile_feedback_msg_.velocity[3] =(average_filter/count)/300.0;
        else
            tactile_feedback_msg_.velocity[3] = 0.0;

        //Middle Base (Palm tactile)
        average_filter = 0;
        count          = 0;
        for(unsigned int i=33; i<48; ++i)
        {
            if(filtered_palm_tactile_msg_[i]>0)
            {
                average_filter+=filtered_palm_tactile_msg_[i];
                count++;
            }
        }
        if(count>0)
            this->tactile_feedback_msg_.velocity[4] = (average_filter/count)/300.0; // was 7
        else
            this->tactile_feedback_msg_.velocity[4] = 0.0;  // was 7

        //Thumb Base (Palm tactile)
        average_filter = 0;
        count          = 0;
        for(unsigned int i=15; i<33; ++i)
        {
            if(filtered_palm_tactile_msg_[i]>0)
            {
                average_filter+=filtered_palm_tactile_msg_[i];
                count++;
            }
        }
        if(count>0)
            this->tactile_feedback_msg_.velocity[4] = (average_filter/count)/300.0;  // was 8
        else
            this->tactile_feedback_msg_.velocity[4] = 0.0;  // was 8


        if(this->tactile_feedback_msg_.velocity[3] > 0.20 ||
           this->tactile_feedback_msg_.velocity[4] > 0.20
            )    // was 6, 7, 8  this->tactile_feedback_msg_.velocity[8] > 0.30
        {
            inner_palm = true;
        }

        for(int i = 0; i < 8; i++)
        {

            if(tactile_feedback_msg_.velocity[i] > 1)
                tactile_feedback_msg_.velocity[i] = 1.0;
            //ROS_INFO("tactile data: %f, %f, %f, %f, %f, %f, %f, %f", tactile_feedback_msg_.velocity[0],tactile_feedback_msg_.velocity[1],tactile_feedback_msg_.velocity[2],
            //         tactile_feedback_msg_.velocity[3],tactile_feedback_msg_.velocity[4],tactile_feedback_msg_.velocity[5],tactile_feedback_msg_.velocity[6],tactile_feedback_msg_.velocity[7]);
        }


        /*
        if (inner_palm && index && middle && thumb)
            return PALM_AND_ALL_FINGERS;
        else if (inner_palm && index && middle)
            return PALM_AND_NO_THUMB;
        else if (inner_palm && thumb && (index || middle) )
            return PALM_AND_THUMB_PLUS_ONE;
        else if (thumb && index && middle)
            return NO_PALM_AND_ALL_FINGERS;
        else if (thumb && (index || middle) )
            return NO_PALM_AND_THUMB_PLUS_ONE;
        else
            return NO_GRASP_QUALITY;
            */
    }

    // transform endeffort to palm pose used by GraspIt
    int VigirIRobotGraspController::staticTransform(geometry_msgs::Pose& palm_pose)
    {
        tf::Transform o_T_hand;    //describes hand in object's frame
        tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

        o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
        o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

        o_T_hand = o_T_pg * gp_T_hand_;

        tf::Quaternion hand_quat;
        tf::Vector3    hand_vector;
        hand_quat   = o_T_hand.getRotation();
        hand_vector = o_T_hand.getOrigin();

        palm_pose.position.x = hand_vector.getX();
        palm_pose.position.y = hand_vector.getY();
        palm_pose.position.z = hand_vector.getZ();
        palm_pose.orientation.x = hand_quat.getX();
        palm_pose.orientation.y = hand_quat.getY();
        palm_pose.orientation.z = hand_quat.getZ();
        palm_pose.orientation.w = hand_quat.getW();
        return 0;
    }


} /// end namespace flor_grasp_controller
