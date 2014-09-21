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

#include <iostream>
#include <boost/thread/locks.hpp>
#include <fstream>
#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <vigir_pick_controller/vigir_robotiq_pick_controller.h>

#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPREAD_RAD     0.28   //Radians range of the spread fingers
#define BYTE_TO_SPR    (SPREAD_RAD/255.0)
#define SPR_TO_BYTE    (1.0/BYTE_TO_SPR)
#define SPR_OFFSET     (BYTE_TO_SPR * 137.0)
#define SPR_OPEN       (-SPR_OFFSET)
#define SPR_CLOSE      SPREAD_RAD-SPR_OFFSET
#define PER_TO_BYTE    2.55

namespace vigir_pick_controller{


    VigirRobotiqPickController::VigirRobotiqPickController()
      : VigirPickController() // explicitly initialize the base class
    {
    }

    VigirRobotiqPickController::~VigirRobotiqPickController()
    {
        std::cout << "Shutting down the Robotiq Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // Robotiq class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirRobotiqPickController::initializeRobotiqPickController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {
      ROS_INFO("Entering initializeRobotiqPickController for %s hand", hand_name_.c_str());
      // Initialize the generic grasp controller components
      initializeGraspController(nh,nhp);

      // Load the hand specific grasping database
      loadRobotiqPickDatabase(this->filename);
    }


    // Hand specific implementations used by the grasp controller

    void VigirRobotiqPickController::processHandSensorData()          // protected by sensor_data and write_data mutex locks
    {
        //if( last_tactile_msg_)
        //    this->local_tactile_msg_     = *(this->last_tactile_msg_);
        //else
        //    ROS_INFO("Hand Tactile Data does not exist");

    }

    void VigirRobotiqPickController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
    {
        tf::Vector3 hand_com = tf::Vector3(0.0,0.0,0.0);
        hand_com.setY(hand_id_ < 1 ? 0.23: -0.23);
        float      hand_mass = 2.3;  // From Robotiq Hand Manual page. 103
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

    void VigirRobotiqPickController::updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type)
    {

    }

    void VigirRobotiqPickController::setHandEfforts(const double& grasp_effort, const double finger_effort[])
    {
        //Effort settings in MANUAL mode
    }
    /////// ---------------------------------- End Hardware callbacks ---------------------------------------

    void VigirRobotiqPickController::loadRobotiqPickDatabase(std::string& file_name)
    {
        /// @TODO - need to add expection protection for reading data out of range

        ROS_INFO("Setup default joint names for the %s hand", hand_name_.c_str());

        // Read the files to load all template and grasping data

        urdf::Model model;
        if (model.initFile(file_name.c_str())){
            ROS_INFO("Successfully parsed urdf file");
        }
        ROS_ERROR("Failed to parse urdf file");

        // TODO - need to make this a paramter that we can set by calling from plugin
        //        std::ifstream file ( file_name.c_str() );
        //        std::string grasp_line;
        //        moveit_msgs::Grasp grasp;
        //        std::string tmp_hand_name;
        //        int8_t      tmp_hand_id;

        //        template_list_.clear();

        //        if (file.is_open())
        //        {
        //          while ( file.good() )
        //          {
        //            getline ( file, grasp_line);
        //            if(grasp_line.find('#')==0 || grasp_line.size() <= 0){
        //                continue;
        //            }
        //            std::stringstream values(grasp_line);
        //            std::string value;
        //            std::stringstream trimmer;
        //            std::getline( values, value, ',' );
        //            grasp_spec.grasp_id = atoi(value.c_str()); //grasp id
        //            if (0 == grasp_spec.grasp_id)
        //            {
        //                ROS_INFO(" Skipping grasp specification 0 for %s given %s controller", tmp_hand_name.c_str(), hand_name_.c_str());
        //                continue;
        //            }

        //            getline( values, value, ',' ); //template type
        //            grasp_spec.template_type = atoi(value.c_str());

        //            getline( values, value, ',' );  //hand
        //            trimmer << value;
        //            value.clear();
        //            trimmer >> value; //removes white spaces
        //            if(value.compare("left") == 0)
        //            {
        //                tmp_hand_name = "l_hand";
        //                tmp_hand_id = -1;
        //            }
        //            else
        //            {
        //              tmp_hand_name = "r_hand";
        //              tmp_hand_id = 1;
        //            }

        //            if (tmp_hand_id != hand_id_)
        //            {
        //                ROS_INFO(" Skipping grasp specification for %s given %s controller", tmp_hand_name.c_str(), hand_name_.c_str());
        //                continue;
        //            }

        //            std::string tmp_grasp_type;
        //            getline( values, tmp_grasp_type, ',' ); //grasp_type
        //            trimmer.clear();
        //            trimmer << tmp_grasp_type;
        //            tmp_grasp_type.clear();
        //            trimmer >> tmp_grasp_type; //removes white spaces
        //            grasp_spec.grasp_type = GRASP_CYLINDRICAL;
        //            if ("spherical" == tmp_grasp_type)
        //            {
        //                grasp_spec.grasp_type = GRASP_SPHERICAL;
        //            } else if ("prismatic" == tmp_grasp_type) {
        //                grasp_spec.grasp_type = GRASP_PRISMATIC;
        //            } else if ("cylindrical" == tmp_grasp_type) {
        //                grasp_spec.grasp_type = GRASP_CYLINDRICAL;
        //            } else {
        //                ROS_WARN(" Unknown grasp type <%s> assuming cylindrical", tmp_grasp_type.c_str());
        //            }

        //            getline( values, value, ',' ); // "finger poses:,"
        //            assert(NUM_ROBOTIQ_PALM_JOINTS == this->joint_names_.size());
        //            for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        //            {
        //              getline( values, value, ',' );

        //              // Graspit outputs the fingers in different order, and these were copied into .grasp library
        //              // We need to swap f0 and f2, which this code does
        //              //grasp_spec.finger_poses.f0[i]= atof(value.c_str()); //joints from the index
        //            }
        //            float ftmp;

        //            getline( values, value, ',' ); // "final pose:,"
        //            for(unsigned int i = 0; i < 7; ++i)
        //            {
        //              getline( values, value, ',' );
        //              ftmp = atof(value.c_str()); //final pose values
        //              if (ftmp > 2.0)
        //              {
        //                  ROS_ERROR("Found pose value > 2.0 in final wrist - Is this file using millimeters again - should be meters!");
        //              }
        //              switch(i)
        //              {
        //                  case 0: grasp_spec.final_wrist_pose.position.x    = ftmp; break;
        //                  case 1: grasp_spec.final_wrist_pose.position.y    = ftmp; break;
        //                  case 2: grasp_spec.final_wrist_pose.position.z    = ftmp; break;
        //                  case 3: grasp_spec.final_wrist_pose.orientation.w = ftmp; break;
        //                  case 4: grasp_spec.final_wrist_pose.orientation.x = ftmp; break;
        //                  case 5: grasp_spec.final_wrist_pose.orientation.y = ftmp; break;
        //                  case 6: grasp_spec.final_wrist_pose.orientation.z = ftmp; break;
        //              }
        //            }
        //            //Static transformation to /r_hand
        //            staticTransform(grasp_spec.final_wrist_pose);

        //            getline( values, value, ',' ); // "pre-grasp pose:,"
        //            for(unsigned int i = 0; i < 7; ++i)
        //            {
        //              getline( values, value, ',' );
        //              ftmp = atof(value.c_str()); //pre-grasp values
        //              if (ftmp > 2.0)
        //              {
        //                  ROS_ERROR("Found pose value > 2.0 in pre-grasp wrist - Is this file using millimeters again - should be meters!");
        //              }
        //              switch(i)
        //              {
        //                  case 0: grasp_spec.pregrasp_wrist_pose.position.x    = ftmp; break;
        //                  case 1: grasp_spec.pregrasp_wrist_pose.position.y    = ftmp; break;
        //                  case 2: grasp_spec.pregrasp_wrist_pose.position.z    = ftmp; break;
        //                  case 3: grasp_spec.pregrasp_wrist_pose.orientation.w = ftmp; break;
        //                  case 4: grasp_spec.pregrasp_wrist_pose.orientation.x = ftmp; break;
        //                  case 5: grasp_spec.pregrasp_wrist_pose.orientation.y = ftmp; break;
        //                  case 6: grasp_spec.pregrasp_wrist_pose.orientation.z = ftmp; break;
        //              }
        //            }
        //            //Static transformation to /r_hand
        //            staticTransform(grasp_spec.pregrasp_wrist_pose);

        //            ROS_INFO("Loading Grasp from file: Id: %d, Template type: %d, Hand name: %s, Hand id: %d, Grasp type: %s",
        //                     grasp_spec.grasp_id,grasp_spec.template_type,tmp_hand_name.c_str(),
        //                     tmp_hand_id,tmp_grasp_type.c_str());
        //            ROS_INFO("      pre-grasp   target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
        //                     grasp_spec.pregrasp_wrist_pose.position.x,    grasp_spec.pregrasp_wrist_pose.position.z,grasp_spec.pregrasp_wrist_pose.position.z,
        //                     grasp_spec.pregrasp_wrist_pose.orientation.w, grasp_spec.pregrasp_wrist_pose.orientation.x, grasp_spec.pregrasp_wrist_pose.orientation.y, grasp_spec.pregrasp_wrist_pose.orientation.z);
        //            ROS_INFO("      final grasp target p=(%f, %f, %f) q=(%f, %f, %f, %f)",
        //                     grasp_spec.final_wrist_pose.position.x,    grasp_spec.final_wrist_pose.position.z,grasp_spec.final_wrist_pose.position.z,
        //                     grasp_spec.final_wrist_pose.orientation.w, grasp_spec.final_wrist_pose.orientation.x, grasp_spec.final_wrist_pose.orientation.y, grasp_spec.final_wrist_pose.orientation.z);
        //            template_list_.push_back(grasp_spec);
        //          }
        //          file.close();
        //        }else
        //        {
        //          ROS_ERROR("Error loading grasp library from : %s",file_name.c_str());
        //        }

        //        // Let's sort the grasp id's into numerical order to allow fast location
        //        ROS_INFO(" Sort grasp identifiers ...");

        //        //std::sort(potential_grasps_.begin(), potential_grasps_.end(), VigirPickSpecificationSorter);

        //        // probably should dump to screen to verify that data is still correct.
        //        ROS_INFO(" Resizing finger poses vectors for grasp types ...");

        ////        initial_finger_poses_.resize(NUM_GRASP_TYPES);
        ////        final_finger_poses_.resize(NUM_GRASP_TYPES);

        ////        ROS_INFO(" Setup initial finger poses for grasp types ...");

        ////        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[0] =0.0;
        ////        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[1] =0.0;
        ////        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[2] =0.0;
        ////        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[3] =0.0;

        ////        initial_finger_poses_.at(GRASP_PRISMATIC).f0[0] =0.0;
        ////        initial_finger_poses_.at(GRASP_PRISMATIC).f0[1] =0.0;
        ////        initial_finger_poses_.at(GRASP_PRISMATIC).f0[2] =0.0;
        ////        initial_finger_poses_.at(GRASP_PRISMATIC).f0[3] =SPR_CLOSE;

        ////        initial_finger_poses_.at(GRASP_SPHERICAL).f0[0] =0.0;
        ////        initial_finger_poses_.at(GRASP_SPHERICAL).f0[1] =0.0;
        ////        initial_finger_poses_.at(GRASP_SPHERICAL).f0[2] =0.0;
        ////        initial_finger_poses_.at(GRASP_SPHERICAL).f0[3] =SPR_OPEN;

        ////        initial_finger_poses_.at(GRASP_NONE).f0[0] =0.0;
        ////        initial_finger_poses_.at(GRASP_NONE).f0[1] =0.0;
        ////        initial_finger_poses_.at(GRASP_NONE).f0[2] =0.0;
        ////        initial_finger_poses_.at(GRASP_NONE).f0[3] =0.0;

        ////        ROS_INFO(" Setup final finger poses for grasp types ...");

        ////        // TODO - define terminal finger positions with fully closed manual grasp
        ////        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[0] =1.22;
        ////        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[1] =1.13;
        ////        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[2] =1.13;
        ////        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[3] =0.0;

        ////        final_finger_poses_.at(GRASP_PRISMATIC).f0[0] =0.53;
        ////        final_finger_poses_.at(GRASP_PRISMATIC).f0[1] =0.49;
        ////        final_finger_poses_.at(GRASP_PRISMATIC).f0[2] =0.49;
        ////        final_finger_poses_.at(GRASP_PRISMATIC).f0[3] =SPR_CLOSE;

        ////        final_finger_poses_.at(GRASP_SPHERICAL).f0[0] =1.22;
        ////        final_finger_poses_.at(GRASP_SPHERICAL).f0[1] =0.58;
        ////        final_finger_poses_.at(GRASP_SPHERICAL).f0[2] =0.58;
        ////        final_finger_poses_.at(GRASP_SPHERICAL).f0[3] =SPR_OPEN;

        ////        final_finger_poses_.at(GRASP_NONE).f0[0] =1.22;
        ////        final_finger_poses_.at(GRASP_NONE).f0[1] =1.13;
        ////        final_finger_poses_.at(GRASP_NONE).f0[2] =1.13;
        ////        final_finger_poses_.at(GRASP_NONE).f0[3] =0.0;

        //        ROS_INFO(" Done setting intital and final poses");

        //        // Initialize states and control values
        //        grasp_id_       = -1 ;
        //        template_id_    = -1 ;
        //        template_type_  = -1 ;
        //        grasp_type_     = GRASP_CYLINDRICAL;
        //        grasp_status_.status  = RobotStatusCodes::GRASP_CONTROLLER_OK;
        //        grasp_status_code_    = RobotStatusCodes::GRASP_CONTROLLER_OK;
        //        grasp_status_severity_= RobotStatusCodes::OK;
        //        ROS_INFO(" Done initializing grasp library! ");

    }



    GraspQuality VigirRobotiqPickController::processHandTactileData()
    {
            return NO_GRASP_QUALITY;
    }

    // transform endeffort to palm pose used by GraspIt
    int VigirRobotiqPickController::staticTransform(geometry_msgs::Pose& palm_pose)
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


} /// end namespace vigir_pick_controller
