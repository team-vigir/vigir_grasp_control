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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sandia_hand_msgs/RawFingerState.h>
#include <vigir_grasp_controllers/vigir_sandia_grasp_controller.h>

namespace vigir_grasp_controller{


    VigirSandiaGraspController::VigirSandiaGraspController()
      : VigirGraspController() // explicitly initialize the base class
    {

        filtered_tactile_msg_.f0.resize(18);
        filtered_tactile_msg_.f1.resize(18);
        filtered_tactile_msg_.f2.resize(18);
        filtered_tactile_msg_.f3.resize(18);
        filtered_tactile_msg_.palm.resize(32);

        for(unsigned int i = 0; i < 18; ++i)
        {
            filtered_tactile_msg_.f0.at(i)=26500;
            filtered_tactile_msg_.f1.at(i)=26500;
            filtered_tactile_msg_.f2.at(i)=26500;
            filtered_tactile_msg_.f3.at(i)=26500;
        }
        for(unsigned int i = 0; i < 32; ++i)
        {
            filtered_tactile_msg_.palm.at(i)=26500;
        }

        // initialize
        local_tactile_msg_ = filtered_tactile_msg_;
    }

    VigirSandiaGraspController::~VigirSandiaGraspController()
    {
        std::cout << "Shutting down the Sandia Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // Sandia class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirSandiaGraspController::initializeSandiaGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {

      // Initialize the generic grasp controller components
      initializeGraspController(nh,nhp);

      hardware_msg_ = false;
      aco_pub_                  = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, true);

      ROS_INFO("Setup parameters and communications for the Sandia Hand grasp controller plugin for %s hand", hand_name_.c_str());
      joint_commands_pub_ = nh.advertise<osrf_msgs::JointCommands>("/sandia_hands/"+hand_name_+"/joint_commands", 1, true);
      //sandia_states_pub_= nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);

      // ros topic subscribtions
      ros::SubscribeOptions jointStatesSo =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/sandia_hands/"+hand_name_+"/joint_states", 1, boost::bind(&VigirSandiaGraspController::jointStatesCallback, this, _1),
              ros::VoidPtr(), nh.getCallbackQueue());

      // Because TCP causes bursty communication with high jitter,
      // declare a preference on UDP connections for receiving
      // joint states, which we want to get at a high rate.
      // Note that we'll still accept TCP connections for this topic
      // (e.g., from rospy nodes, which don't support UDP);
      // we just prefer UDP.
      jointStatesSo.transport_hints = ros::TransportHints().unreliable();

      measured_joint_state_sub_ = nh.subscribe(jointStatesSo);

      // Subscribe to tactile
      ros::SubscribeOptions tacSo =
      ros::SubscribeOptions::create<sandia_hand_msgs::RawTactile>(
          "/sandia_hands/"+hand_name_+"/tactile_raw", 1, boost::bind(&VigirSandiaGraspController::tactileCallback, this, _1),
          ros::VoidPtr(), nh.getCallbackQueue());
      tacSo.transport_hints = ros::TransportHints().unreliable();
      hand_tactile_sub_ = nh.subscribe(tacSo);

      //<sandia_hand_msgs::RawFingerState>
      ros::SubscribeOptions f0So =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/joint_states", 1, boost::bind(&VigirSandiaGraspController::finger_0_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      finger_0_sub_ = nh.subscribe(f0So);
/*
      ros::SubscribeOptions f1So =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_controller/" + hand_name_ + "/joint_states", 1, boost::bind(&VigirSandiaGraspController::finger_1_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      finger_1_sub_ = nh.subscribe(f1So);

      ros::SubscribeOptions f2So =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_controller/" + hand_name_ + "/joint_states", 1, boost::bind(&VigirSandiaGraspController::finger_2_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      finger_2_sub_ = nh.subscribe(f2So);

      ros::SubscribeOptions f3So =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_controller/" + hand_name_ + "/joint_states", 1, boost::bind(&VigirSandiaGraspController::finger_3_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      finger_3_sub_ = nh.subscribe(f3So);
*/
      //finger_1_sub_ = nh.subscribe("/sandia_hands/" +hand_name_+"/finger_1/raw_states", 1, VigirSandiaGraspController::finger_1_Callback);
      //finger_2_sub_ = nh.subscribe("/sandia_hands/" +hand_name_+"/finger_2/raw_states", 1, VigirSandiaGraspController::finger_2_Callback);
      //finger_3_sub_ = nh.subscribe("/sandia_hands/" +hand_name_+"/finger_3/raw_states", 1, VigirSandiaGraspController::finger_3_Callback);

      // Load the hand specific grasping database
      loadSandiaGraspDatabase(this->filename);

      ///////////////////////////////////////////
      // This section added for Sandia Hardware callbacks
      local_joint_state_msg_.name.resize(12);
      local_joint_state_msg_.position.resize(12);
      local_joint_state_msg_.velocity.resize(12);
      local_joint_state_msg_.effort.resize(12);

      local_joint_state_msg_.name = joint_names_;
      ////////////////////////////////////////////

    }


    // Hand specific implementations used by the grasp controller

    //
    void VigirSandiaGraspController::ZeroHandJointCommands()
    {
        for (unsigned i = 0; i < this->joint_names_.size(); ++i)
        {
          this->joint_commands_.position[i] = 0;
          this->joint_commands_.velocity[i] = 0;
          this->joint_commands_.effort[i] = 0;
          this->joint_commands_.kp_position[i] = 0;
          this->joint_commands_.ki_position[i] = 0;
          this->joint_commands_.kd_position[i] = 0;
          this->joint_commands_.kp_velocity[i] = 0;
          this->joint_commands_.i_effort_min[i] = 0;
          this->joint_commands_.i_effort_max[i] = 0;
        }
        return;

    }

    void VigirSandiaGraspController::processHandSensorData()          // protected by sensor_data and write_data mutex locks
    {
        // Store the ROS message data into local structures for thread safe access
        if (last_joint_state_msg_)
            this->local_joint_state_msg_ = *(this->last_joint_state_msg_);
        //else
        //    ROS_INFO("Hand Joint Data does not exist");

        if( last_tactile_msg_)
            this->local_tactile_msg_     = *(this->last_tactile_msg_);
        //else
        //    ROS_INFO("Hand Tactile Data does not exist");

    }

    void VigirSandiaGraspController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
    {
        tf::Vector3 hand_com = tf::Vector3(0.0,0.0,0.0);
        hand_com.setY(hand_id_ < 1 ? 0.23: -0.23);
        float      hand_mass = 2.96;
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
            hand_com.setY(hand_id_ < 1 ? 0.23: -0.23);
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
            com_.pose.position.y = hand_id_ < 1 ? 0.23: -0.23;
            com_.pose.position.z = 0.0;
        }
        this->hand_mass_msg_.com = com_;
    }

    void VigirSandiaGraspController::publishHandStates(bool update_joint_commands)
    {
       // call as needed to publish the updated joint commands
       if (update_joint_commands)
       {
           // Set the next joint state header
           this->joint_commands_.header.seq++;
           this->joint_commands_.header.stamp    = this->local_joint_state_msg_.header.stamp;
           this->joint_commands_.header.frame_id = this->local_joint_state_msg_.header.frame_id;
           joint_commands_pub_.publish(joint_commands_);
           //sandia_states_pub_.publish();
       }


       // @todo - for Physical hand, should we load sensor data into simulation structures and publish to mimic simulation?
       //   who in OCS/onboard is using that form of the data?

    }

    void VigirSandiaGraspController::setInitialFingerPoses(const uint8_t& grasp_type)
    {
        if (grasp_type < this->initial_finger_poses_.size())
        {
            //ROS_ERROR("Use grasp type=%d to set the standard finger poses = initial", grasp_type);
            this->finger_poses_  = this->initial_finger_poses_.at(grasp_type);
        }
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
    }

    void VigirSandiaGraspController::setFinalFingerPoses(const uint8_t& grasp_type)
    {
        if (grasp_type < this->final_finger_poses_.size())
            this->finger_poses_  = this->final_finger_poses_.at(grasp_type);
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
    }

    void VigirSandiaGraspController::setInitialJointToCurrentPositions()
    {
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
            real_initial_finger_poses_.fingers[i] = this->local_joint_state_msg_.position[i]; //stores real joint positions
            ROS_INFO("Real finger %d joint %d pose: %f", i/3, i%3, real_initial_finger_poses_.fingers[i]);
        }
    }

    void VigirSandiaGraspController::setFingerPosesToID(const uint16_t& requested_release_id, const uint16_t& grasp_template_id)
    {// Assumes data is locked by calling thread
        for (std::vector <VigirSandiaGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
        {
            if((*it).grasp_id == requested_release_id)
            {
                this->grasp_id_       = (*it).grasp_id;
                this->template_id_    = grasp_template_id;
                this->template_type_  = (*it).template_type;
                this->finger_poses_   = (*it).finger_poses;   // assumes defined array fully copied
            }
        }

        if ( (this->grasp_id_ != requested_release_id))
        {
            ROS_WARN("%s: Release grasp selection id: %d not found.",
                     hand_name_.c_str(), requested_release_id);
        }

    }

    void VigirSandiaGraspController::updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type)
    {
        for (std::vector <VigirSandiaGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
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

//                            ROS_INFO("  update grasp id = %d template=%d ", this->grasp_id_, this->template_id_);
//                            ROS_INFO("        update grasp final-grasp target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//                                     this->final_wrist_pose_.position.x,    this->final_wrist_pose_.position.z,    this->final_wrist_pose_.position.z,
//                                     this->final_wrist_pose_.orientation.w, this->final_wrist_pose_.orientation.x, this->final_wrist_pose_.orientation.y, this->final_wrist_pose_.orientation.z);
//                            ROS_INFO("        update grasp pre-grasp target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//                                     this->pregrasp_wrist_pose_.position.x,    this->pregrasp_wrist_pose_.position.z,    this->pregrasp_wrist_pose_.position.z,
//                                     this->pregrasp_wrist_pose_.orientation.w, this->pregrasp_wrist_pose_.orientation.x, this->pregrasp_wrist_pose_.orientation.y, this->pregrasp_wrist_pose_.orientation.z);

           }

        }

        if (-1 == this->grasp_id_ ) ROS_WARN("Failed to match grasp ID - set to -1");

    }

    void VigirSandiaGraspController::setAttachingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data)
    {
    }

    void VigirSandiaGraspController::setDetachingObject( )
    {
    }

    void VigirSandiaGraspController::setHandEfforts(const double& grasp_effort, const double finger_effort[])
    {
        //Effort settings in MANUAL mode
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
           switch(i){
           case 1:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[1]*0.03; //Index Proximal
               break;
           case 2:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[1]*0.01; //Index Distal
               break;
           case 4:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[2]*0.03; //Middle Proximal
               break;
           case 5:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[2]*0.01; //Middle Distal
               break;
           case 7:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[3]*0.03; //Pinky proximal
               break;
           case 8:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[3]*0.01; //Pinky Distal
               break;
           case 10: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[0]*0.03; //Thumb Proximal
               break;
           case 11: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[0]*0.01; //Thumb Distal
               break;
           default: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.002;
               break;
           }

        }
    }

    void VigirSandiaGraspController::setHandNoneData( )
    {
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
            // Assumes that data is aligned, and we just move from buffer to next through f3[2]
            current_finger_poses_.fingers[i] = this->initial_finger_poses_.at(GRASP_NONE).fingers[i];
            this->joint_commands_.position[i] = current_finger_poses_.fingers[i];
            this->joint_commands_.effort[i]   = 0.0;
        }
    }

    void VigirSandiaGraspController::setHandApproachingData(const double& grasp_fraction)
    {
        double one_minus_fraction = 1.0 - grasp_fraction;

        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.fingers[i] = real_initial_finger_poses_.fingers[i] * one_minus_fraction +  //moving from the actual joint states avoids jumping from a full close to a full open, or between any two different grasps.
                                             initial_finger_poses_.at(this->grasp_type_).fingers[i] * grasp_fraction;
           this->joint_commands_.position[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.effort[i]   = 0.0;
        }

    }
    void VigirSandiaGraspController::setHandSurroundingData( )
    {
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i]; // waste to recalc, but otherwise must keep track of transition
           this->joint_commands_.position[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.effort[i]   = 0.0;
        }
    }

    void VigirSandiaGraspController::setHandGraspingData(const double& grasp_fraction, const int8_t finger_effort[])
    {

        //ROS_INFO("setHandGraspingData grasp_fraction=%f",grasp_fraction);
        double one_minus_fraction = 1.0 - grasp_fraction;
        double grasp_fraction_f1  = grasp_fraction * ratio_f1_f2;
        if(grasp_fraction_f1 > 1.0)
           grasp_fraction_f1 = 1.0;
        double one_minus_frac_f1  = 1.0 - grasp_fraction_f1;
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {

           switch(i){
           case 1:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_frac_f1 +
                       this->finger_poses_.fingers[i] * grasp_fraction_f1+finger_effort[1]*0.03; //Index Proximal
               break;
           case 2:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction+finger_effort[1]*0.01; //Index Distal
               break;
           case 4:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_frac_f1 +
                       this->finger_poses_.fingers[i] * grasp_fraction_f1+finger_effort[2]*0.03; //Middle Proximal
               break;
           case 5:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction+finger_effort[2]*0.01; //Middle Distal
               break;
           case 7:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_frac_f1 +
                       this->finger_poses_.fingers[i] * grasp_fraction_f1+finger_effort[3]*0.03; //Pinky proximal
               break;
           case 8:  current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction+finger_effort[3]*0.01; //Pinky Distal
               break;
           case 10: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction+finger_effort[0]*0.03; //Thumb Proximal
               break;
           case 11: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction_f1+finger_effort[0]*0.01; //Thumb Distal
               break;
           default: current_finger_poses_.fingers[i] = initial_finger_poses_.at(this->grasp_type_).fingers[i] * one_minus_fraction +
                       this->finger_poses_.fingers[i] * grasp_fraction;
               break;
           }

           //std::cout << "        f" << i << "  position=" << current_finger_poses_.f0[i] << " = " << initial_finger_poses_.at(this->grasp_type_).f0[i] << " + " << this->finger_poses_.f0[i]<< std::endl;
           this->joint_commands_.position[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.effort[i]   = 0.0;
           //this->joint_commands_.effort[i]   = grasp_effort*0.001; // @TODO - fix the scaling on this


           // Increase joint angles when using hardware to immitate feedforward effort
           if(hardware_msg_)
           {
               switch(i){
               case 1:  this->joint_commands_.position[i] +=  finger_effort[1] * 0.003 * 3 ; //Index Proximal
                   break;
               case 2:  this->joint_commands_.position[i] +=  finger_effort[1] * 0.001 * 3; //Index Distal
                   break;
               case 4:  this->joint_commands_.position[i] +=  finger_effort[2] * 0.003 * 3 ; //Middle Proximal
                   break;
               case 5:  this->joint_commands_.position[i] +=  finger_effort[2] * 0.001 * 3; //Middle Distal
                   break;
               case 7:  this->joint_commands_.position[i] +=  finger_effort[3] * 0.003 * 3 ; //Pinky proximal
                   break;
               case 8:  this->joint_commands_.position[i] +=  finger_effort[3] * 0.001 * 3; //Pinky Distal
                   break;
               case 10: this->joint_commands_.position[i] +=  finger_effort[0] * 0.003 * 3 ; //Thumb Proximal
                   break;
               case 11: this->joint_commands_.position[i] +=  finger_effort[0] * 0.001 * 3; //Thumb Distal
                   break;
               default:
                   break;
               }
           }

        }
    }

    void VigirSandiaGraspController::setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])
    {
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
            // Assumes that data is aligned, and we just move from buffer to next through f3[2]
            current_finger_poses_.fingers[i] = this->finger_poses_.fingers[i]; // waste to recalc, but otherwise must keep track of transition
            this->joint_commands_.position[i] = current_finger_poses_.fingers[i];

            switch(i){
            case 1:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[1]*0.03; //Index Proximal
                break;
            case 2:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[1]*0.01; //Index Distal
                break;
            case 4:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[2]*0.03; //Middle Proximal
                break;
            case 5:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[2]*0.01; //Middle Distal
                break;
            case 7:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[3]*0.03; //Pinky proximal
                break;
            case 8:  this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[3]*0.01; //Pinky Distal
                break;
            case 10: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[0]*0.03; //Thumb Proximal
                break;
            case 11: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.03+finger_effort[0]*0.01; //Thumb Distal
                break;
            default: this->joint_commands_.effort[i] = 0.00001 + grasp_effort*0.002;
                break;
            }

            // Increase joint angles when using hardware to immitate feedforward effort
            if(hardware_msg_)
            {
                int ff_effort = 3;
                switch(i){
                case 1:  this->joint_commands_.position[i] += grasp_effort * 0.003 * ff_effort + finger_effort[1] * 0.003 * ff_effort; //Index Proximal
                    break;
                case 2:  this->joint_commands_.position[i] += grasp_effort * 0.001 * ff_effort + finger_effort[1] * 0.001 * ff_effort; //Index Distal
                    break;
                case 4:  this->joint_commands_.position[i] += grasp_effort * 0.003 * ff_effort + finger_effort[2] * 0.003 * ff_effort; //Middle Proximal
                    break;
                case 5:  this->joint_commands_.position[i] += grasp_effort * 0.001 * ff_effort + finger_effort[2] * 0.001 * ff_effort; //Middle Distal
                    break;
                case 7:  this->joint_commands_.position[i] += grasp_effort * 0.003 * ff_effort + finger_effort[3] * 0.003 * ff_effort; //Pinky proximal
                    break;
                case 8:  this->joint_commands_.position[i] += grasp_effort * 0.001 * ff_effort + finger_effort[3] * 0.001 * ff_effort; //Pinky Distal
                    break;
                case 10: this->joint_commands_.position[i] += grasp_effort * 0.003 * ff_effort + finger_effort[0] * 0.003 * ff_effort; //Thumb Proximal
                    break;
                case 11: this->joint_commands_.position[i] += grasp_effort * 0.001 * ff_effort + finger_effort[0] * 0.001 * ff_effort; //Thumb Distal
                    break;
                default: this->joint_commands_.position[i] += grasp_effort * 0.0002 * ff_effort;
                    break;
                }
            }
        }
    }

    void VigirSandiaGraspController::setHandOpeningData(const double& grasp_fraction)
    {
        double one_minus_fraction = 1.0 - grasp_fraction;
        for(unsigned int i = 0; i < NUM_SANDIA_FINGER_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.fingers[i] = real_initial_finger_poses_.fingers[i] * one_minus_fraction +
                                                initial_finger_poses_.at(this->grasp_type_).fingers[i] * grasp_fraction;
           //ROS_INFO("Setting finger %d joint %d to:  %f", i/3,i%3,current_finger_poses_.f0[i]);
           this->joint_commands_.position[i] = current_finger_poses_.fingers[i];
           this->joint_commands_.effort[i]   = 0.0;
        }
    }

    /////// ---------------------------------- Simulation callbacks ---------------------------------------
    // Hand specific callbacks used for processing simulation data
    void VigirSandiaGraspController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &js_msg)
    {
      boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
      last_joint_state_msg_ = js_msg;

      // This call triggers processing of data and handling grasp calculations with worker thread loop
      this->updatedSensorData();
    }

    void VigirSandiaGraspController::tactileCallback(const sandia_hand_msgs::RawTactile::ConstPtr &tac_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_tactile_msg_ = tac_msg;
    }
    /////// ---------------------------------- End Simulation callbacks ---------------------------------------

    /////// ---------------------------------- Hardware callbacks ---------------------------------------


    void VigirSandiaGraspController::finger_0_Callback(const sensor_msgs::JointStateConstPtr& finger_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        std::string finger1, finger2, finger3, finger4;
        if(hand_name_ == "r_hand")
        {
            finger1 = "right_f0_j0";
            finger2 = "right_f1_j0";
            finger3 = "right_f2_j0";
            finger4 = "right_f3_j0";
        }
        else
        {
            finger1 = "left_f0_j0";
            finger2 = "left_f1_j0";
            finger3 = "left_f2_j0";
            finger4 = "left_f3_j0";
        }
           // l_hand or r_hand
        int i;

        if(finger_msg->name[0] == finger1)
            i = 0;
        if(finger_msg->name[0] == finger2)
            i = 3;
        if(finger_msg->name[0] == finger3)
            i = 6;
        if(finger_msg->name[0] == finger4)
            i = 9;

        for(int j = 0; j < 3; j++)
            local_joint_state_msg_.position[j+i] = finger_msg->position[j];
        local_joint_state_msg_.header.stamp = ros::Time::now();
        this->updatedSensorData();

        if(hardware_msg_ == false)
          hardware_msg_ = true;
    }
/*
    void VigirSandiaGraspController::finger_1_Callback(const sensor_msgs::JointStateConstPtr& finger_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        for(int i = 0; i < 3; i++)
            local_joint_state_msg_.position[i+3] = finger_msg->hall_pos[i];
        local_joint_state_msg_.header.stamp = ros::Time::now();
        this->updatedSensorData();

        if(hardware_msg_ == false)
          hardware_msg_ = true;
    }

    void VigirSandiaGraspController::finger_2_Callback(const sensor_msgs::JointStateConstPtr& finger_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        for(int i = 0; i < 3; i++)
            local_joint_state_msg_.position[i+6] = finger_msg->hall_pos[i];
        local_joint_state_msg_.header.stamp = ros::Time::now();
        this->updatedSensorData();

        if(hardware_msg_ == false)
          hardware_msg_ = true;
    }

    void VigirSandiaGraspController::finger_3_Callback(const sandia_hand_msgs::RawFingerStateConstPtr& finger_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        for(int i = 0; i < 3; i++)
            local_joint_state_msg_.position[i+9] = finger_msg->hall_pos[i];
        local_joint_state_msg_.header.stamp = ros::Time::now();
        this->updatedSensorData();

        if(hardware_msg_ == false)
          hardware_msg_ = true;
    }
    */
    /////// ---------------------------------- End Hardware callbacks ---------------------------------------

    // Hand specific messages for handling data
    bool FlorGraspSpecificationSorter(VigirSandiaGraspSpecification const& a, VigirSandiaGraspSpecification const& b)
    {
        if (a.grasp_id < b.grasp_id) return true;
        return false;
    }

    void VigirSandiaGraspController::loadSandiaGraspDatabase(std::string& file_name)
    {
        /// @TODO - need to add expection protection for reading data out of range

        ROS_INFO("Setup default joint names for the %s hand", hand_name_.c_str());

        // Read the files to load all template and grasping data

        // TODO - need to make this a paramter that we can set by calling from plugin
        std::ifstream file ( file_name.c_str() );
        std::string grasp_line;
        VigirSandiaGraspSpecification grasp_spec;
        std::string tmp_hand_name;
        int8_t      tmp_hand_id;

        potential_grasps_.clear();
        potential_grasps_.push_back(VigirSandiaGraspSpecification()); // default 0 grasp


        this->joint_names_.clear(); // reset the vector

        // must match those inside of the /sandia_hands/?_hand/joint_states/[right_/left_]+
        if(hand_id_ < 0)
        {
            tmp_hand_name = "left_";
        }
        else
        {
            tmp_hand_name = "right_";
        }
        this->joint_names_.push_back(tmp_hand_name+"f0_j0");
        this->joint_names_.push_back(tmp_hand_name+"f0_j1");
        this->joint_names_.push_back(tmp_hand_name+"f0_j2");
        this->joint_names_.push_back(tmp_hand_name+"f1_j0");
        this->joint_names_.push_back(tmp_hand_name+"f1_j1");
        this->joint_names_.push_back(tmp_hand_name+"f1_j2");
        this->joint_names_.push_back(tmp_hand_name+"f2_j0");
        this->joint_names_.push_back(tmp_hand_name+"f2_j1");
        this->joint_names_.push_back(tmp_hand_name+"f2_j2");
        this->joint_names_.push_back(tmp_hand_name+"f3_j0");
        this->joint_names_.push_back(tmp_hand_name+"f3_j1");
        this->joint_names_.push_back(tmp_hand_name+"f3_j2");

        this->joint_commands_.position.resize(this->joint_names_.size());
        this->joint_commands_.effort.resize(this->joint_names_.size());

        if (NUM_SANDIA_FINGER_JOINTS != this->joint_names_.size())
        {   // should throw exception here and abort plugin loading
            ROS_ERROR("Invalid joint names - don't match our vectors %d vs. %d", uint8_t(this->joint_names_.size()), NUM_SANDIA_FINGER_JOINTS);
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
            assert(NUM_SANDIA_FINGER_JOINTS == this->joint_names_.size());
            for(unsigned int i = 0; i < this->joint_names_.size(); ++i)
            {
              getline( values, value, ',' );

              // Graspit outputs the fingers in different order, and these were copied into .grasp library
              // We need to swap f0 and f2, which this code does
              if(i<3)
                grasp_spec.finger_poses.fingers[i+6]= atof(value.c_str()); //joints from the pinky
              else if (i>5 && i<9)
                grasp_spec.finger_poses.fingers[i-6]= atof(value.c_str()); //joints from the index
              else if (i>2 && i<6)
                grasp_spec.finger_poses.fingers[i]= atof(value.c_str()); //joints from middle
              else //9,10,11
                grasp_spec.finger_poses.fingers[i]= atof(value.c_str()); //joints from thumb

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

        // probably should dump to screen to verify that data is still correct.
        ROS_INFO(" Setup initial finger poses for grasp types ...");
        initial_finger_poses_.resize(NUM_GRASP_TYPES);
        final_finger_poses_.resize(NUM_GRASP_TYPES);

        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[0]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[1]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[2]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[4]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[5]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[6]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[7]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[8]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[9]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[10]=0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).fingers[11]=0.0;

        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[0]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[1]=0.3;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[2]=1.57;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[4]=0.3;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[5]=1.57;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[6]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[7]=0.3;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[8]=1.57;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[9]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[10]=0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).fingers[11]=1.57;

        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[0]=-0.8;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[1]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[2]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[3]=0.32;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[4]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[5]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[6]=1.35;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[7]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[8]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[9]=-0.01;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[10]=0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).fingers[11]=0.0;

        initial_finger_poses_.at(GRASP_NONE).fingers[0]=0.0;
        initial_finger_poses_.at(GRASP_NONE).fingers[1]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[2]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[3]=0.0;
        initial_finger_poses_.at(GRASP_NONE).fingers[4]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[5]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[6]=0.0;
        initial_finger_poses_.at(GRASP_NONE).fingers[7]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[8]=1.57;
        initial_finger_poses_.at(GRASP_NONE).fingers[9]=1.2;
        initial_finger_poses_.at(GRASP_NONE).fingers[10]=1.0;
        initial_finger_poses_.at(GRASP_NONE).fingers[11]=0.3;

        // TODO - define terminal finger positions with fully closed manual grasp
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[0]=0.0;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[1]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[2]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[4]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[5]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[6]=0.0;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[7]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[8]=1.57;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[9]=0.0;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[10]=0.7;
        final_finger_poses_.at(GRASP_CYLINDRICAL).fingers[11]=0.3;

        final_finger_poses_.at(GRASP_PRISMATIC).fingers[0]=0.33;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[1]=1.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[2]=0.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[3]=0.33;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[4]=1.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[5]=0.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[6]=0.33;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[7]=1.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[8]=0.57;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[9]=-0.01;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[10]=1.06;
        final_finger_poses_.at(GRASP_PRISMATIC).fingers[11]=-0.19;

        final_finger_poses_.at(GRASP_SPHERICAL).fingers[0]=-0.64;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[1]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[2]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[3]=0.40;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[4]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[5]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[6]=1.33;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[7]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[8]=1.57;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[9]=0.0;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[10]=0.7;
        final_finger_poses_.at(GRASP_SPHERICAL).fingers[11]=0.3;

        final_finger_poses_.at(GRASP_NONE).fingers[0]=0.0;
        final_finger_poses_.at(GRASP_NONE).fingers[1]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[2]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[3]=0.0;
        final_finger_poses_.at(GRASP_NONE).fingers[4]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[5]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[6]=0.0;
        final_finger_poses_.at(GRASP_NONE).fingers[7]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[8]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[9]=0.0;
        final_finger_poses_.at(GRASP_NONE).fingers[10]=-1.57;
        final_finger_poses_.at(GRASP_NONE).fingers[11]=-1.57;

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



    GraspQuality VigirSandiaGraspController::processHandTactileData()
    {

        bool thumb      = false,
             index      = false,
             middle     = false,
             pinky      = false,
             inner_palm = false;

        unsigned int count      = 0;
        uint16_t average_filter = 0.0;

        for(unsigned int i = 0; i < 18; ++i)
        {
            this->filtered_tactile_msg_.f0[i]=this->filtered_tactile_msg_.f0[i]*0.9+(this->local_tactile_msg_.f0[i]-26500)*0.1;
            this->filtered_tactile_msg_.f1[i]=this->filtered_tactile_msg_.f1[i]*0.9+(this->local_tactile_msg_.f1[i]-26500)*0.1;
            this->filtered_tactile_msg_.f2[i]=this->filtered_tactile_msg_.f2[i]*0.9+(this->local_tactile_msg_.f2[i]-26500)*0.1;
            this->filtered_tactile_msg_.f3[i]=this->filtered_tactile_msg_.f3[i]*0.9+(this->local_tactile_msg_.f3[i]-26500)*0.1;
        }
        for(unsigned int i = 0; i < 32; ++i)
        {
            this->filtered_tactile_msg_.palm[i]=this->filtered_tactile_msg_.palm[i]*0.9+(this->local_tactile_msg_.palm[i]-26500)*0.1;
        }

        for(unsigned int i=0; i<6; ++i)  //Index proximal
        {
            if(filtered_tactile_msg_.f0[i]>0)
            {
                index = true;
                average_filter+=filtered_tactile_msg_.f0[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[1]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[1]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=6; i<18; ++i) //Index distal
        {
            if(filtered_tactile_msg_.f0[i]>0)
            {
                index = true;
                average_filter+=filtered_tactile_msg_.f0[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[2]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[2]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=0; i<6; ++i) //Middle proximal
        {
            if(filtered_tactile_msg_.f1[i]>0)
            {
                middle = true;
                average_filter+=filtered_tactile_msg_.f1[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[4]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[4]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=6; i<18; ++i) //Middle distal
        {
            if(filtered_tactile_msg_.f1[i]>0)
            {
                middle = true;
                average_filter+=filtered_tactile_msg_.f1[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[5]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[5]=0;


        count          = 0;
        average_filter = 0;
        for(unsigned int i=0; i<6; ++i) //Pinky proximal
        {
            if(filtered_tactile_msg_.f2[i]>0)
            {
                pinky = true;
                average_filter+=filtered_tactile_msg_.f2[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[7]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[7]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=6; i<18; ++i) //Pinky distal
        {
            if(filtered_tactile_msg_.f2[i]>0)
            {
                pinky = true;
                average_filter+=filtered_tactile_msg_.f2[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[8]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[8]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=0; i<6; ++i) //Thumb proximal
        {
            if(filtered_tactile_msg_.f3[i]>0)
            {
                thumb = true;
                average_filter+=filtered_tactile_msg_.f3[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[10]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[10]=0;

        count          = 0;
        average_filter = 0;
        for(unsigned int i=6; i<18; ++i) //Thumb distal
        {
            if(filtered_tactile_msg_.f3[i]>0)
            {
                thumb = true;
                average_filter+=filtered_tactile_msg_.f3[i];
                count++;
            }
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[11]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[11]=0;



        //Index Base
        average_filter = 0;
        count          = 0;
        if(filtered_tactile_msg_.palm[0]>0){
            average_filter += filtered_tactile_msg_.palm[0];
            count++;
        }
        if(filtered_tactile_msg_.palm[3]>0){
            average_filter += filtered_tactile_msg_.palm[3];
            count++;
        }
        if(filtered_tactile_msg_.palm[4]>0){
            average_filter += filtered_tactile_msg_.palm[4];
            count++;
        }
        if(filtered_tactile_msg_.palm[9]>0){
            average_filter += filtered_tactile_msg_.palm[9];
            count++;
        }
        if(filtered_tactile_msg_.palm[13]>0){
            average_filter += filtered_tactile_msg_.palm[13];
            count++;
        }
        if(filtered_tactile_msg_.palm[17]>0){
            average_filter += filtered_tactile_msg_.palm[17];
            count++;
        }
        if(filtered_tactile_msg_.palm[18]>0){
            average_filter += filtered_tactile_msg_.palm[18];
            count++;
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[0]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[0]=0;

        //Middle Base
        average_filter = 0;
        count          = 0;
        if(filtered_tactile_msg_.palm[1]>0){
            average_filter += filtered_tactile_msg_.palm[1];
            count++;
        }
        if(filtered_tactile_msg_.palm[5]>0){
            average_filter += filtered_tactile_msg_.palm[5];
            count++;
        }
        if(filtered_tactile_msg_.palm[6]>0){
            average_filter += filtered_tactile_msg_.palm[6];
            count++;
        }
        if(filtered_tactile_msg_.palm[10]>0){
            average_filter += filtered_tactile_msg_.palm[10];
            count++;
        }
        if(filtered_tactile_msg_.palm[11]>0){
            average_filter += filtered_tactile_msg_.palm[11];
            count++;
        }
        if(filtered_tactile_msg_.palm[14]>0){
            average_filter += filtered_tactile_msg_.palm[14];
            count++;
        }
        if(filtered_tactile_msg_.palm[15]>0){
            average_filter += filtered_tactile_msg_.palm[15];
            count++;
        }
        if(filtered_tactile_msg_.palm[19]>0){
            average_filter += filtered_tactile_msg_.palm[19];
            count++;
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[3]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[3]=0;

        //Pinky Base
        average_filter = 0;
        count          = 0;
        if(filtered_tactile_msg_.palm[2]>0){
            average_filter += filtered_tactile_msg_.palm[2];
            count++;
        }
        if(filtered_tactile_msg_.palm[7]>0){
            average_filter += filtered_tactile_msg_.palm[7];
            count++;
        }
        if(filtered_tactile_msg_.palm[8]>0){
            average_filter += filtered_tactile_msg_.palm[8];
            count++;
        }
        if(filtered_tactile_msg_.palm[12]>0){
            average_filter += filtered_tactile_msg_.palm[12];
            count++;
        }
        if(filtered_tactile_msg_.palm[16]>0){
            average_filter += filtered_tactile_msg_.palm[16];
            count++;
        }
        if(filtered_tactile_msg_.palm[20]>0){
            average_filter += filtered_tactile_msg_.palm[20];
           count++;
        }
        if(filtered_tactile_msg_.palm[21]>0){
            average_filter += filtered_tactile_msg_.palm[21];
            count++;
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[6]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[6]=0;

        //Thumb Base
        average_filter = 0;
        count          = 0;
        if(filtered_tactile_msg_.palm[22]>0){
            average_filter += filtered_tactile_msg_.palm[22];
            count++;
        }
        if(filtered_tactile_msg_.palm[23]>0){
            average_filter += filtered_tactile_msg_.palm[23];
            count++;
        }
        if(filtered_tactile_msg_.palm[24]>0){
            average_filter += filtered_tactile_msg_.palm[24];
            count++;
        }
        if(filtered_tactile_msg_.palm[25]>0){
            average_filter += filtered_tactile_msg_.palm[25];
            count++;
        }
        if(filtered_tactile_msg_.palm[26]>0){
            average_filter += filtered_tactile_msg_.palm[26];
            count++;
        }
        if(filtered_tactile_msg_.palm[27]>0){
            average_filter += filtered_tactile_msg_.palm[27];
            count++;
        }
        if(filtered_tactile_msg_.palm[28]>0){
            average_filter += filtered_tactile_msg_.palm[28];
            count++;
        }
        if(filtered_tactile_msg_.palm[29]>0){
            average_filter += filtered_tactile_msg_.palm[29];
            count++;
        }
        if(filtered_tactile_msg_.palm[30]>0){
            average_filter += filtered_tactile_msg_.palm[30];
            count++;
        }
        if(filtered_tactile_msg_.palm[31]>0){
            average_filter += filtered_tactile_msg_.palm[31];
            count++;
        }
        if(count>0)
            this->local_joint_state_msg_.velocity[9]=(average_filter/count)/7000.0;
        else
            this->local_joint_state_msg_.velocity[9]=0;

        if(this->local_joint_state_msg_.velocity[0] > 0 ||
           this->local_joint_state_msg_.velocity[3] > 0 ||
           this->local_joint_state_msg_.velocity[6] > 0 ||
           this->local_joint_state_msg_.velocity[9] > 0 )
        {
            inner_palm = true;
        }
        if (inner_palm && index && middle && pinky && thumb)
            return PALM_AND_ALL_FINGERS;
        else if (inner_palm && index && middle && pinky)
            return PALM_AND_NO_THUMB;
        else if (inner_palm && ((index && middle) || (index && pinky) || (middle && pinky)))
            return PALM_AND_NO_THUMB_LESS_ONE;
        else if (thumb && index && middle && pinky)
            return NO_PALM_AND_ALL_FINGERS;
        else if (thumb && ((index && middle) || (index && pinky) || (middle && pinky)))
            return NO_PALM_AND_THUMB_PLUS_TWO;
        else
            return NO_GRASP_QUALITY;
    }

    // transform endeffort to palm pose used by GraspIt
    int VigirSandiaGraspController::staticTransform(geometry_msgs::Pose& palm_pose)
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
