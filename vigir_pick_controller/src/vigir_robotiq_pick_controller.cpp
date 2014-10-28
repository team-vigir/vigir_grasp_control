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

      // Load the Object Template database
      VigirPickController::loadObjectTemplateDatabase(this->ot_filename_);

      // Load the hand specific grasping database
      loadRobotiqPickDatabase(this->filename_);
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

        // Read the files to load all grasping data


        std::vector< std::vector <std::string> > db = VigirPickController::readCSVFile(file_name);
        //# grasp id, template type, hand, initial grasp type, finger joints (12), final grasp pose relative to template (x,y,z,qw,qx,qy,qz), pre-grasp pose relative to template (x,y,z,qw,qx,qy,qz)


        for(int i = 1; i < db.size(); i++) //STARTING FROM 1 SINCE FIRST LINE IS HEADER BEGINING WITH "#"
        {
            if(db[i][2].compare(this->hand_side_) != 0)
            {
                ROS_INFO(" Skipping grasp specification for %s given %s controller", db[i][9].c_str(), hand_name_.c_str());
                continue;
            }
            unsigned int type = std::atoi(db[i][1].c_str());

            moveit_msgs::Grasp grasp;
            grasp.id                            = db[i][0];
            grasp.grasp_pose.header.frame_id    = this->hand_name_;
            grasp.grasp_pose.pose.position.x    = std::atof(db[i][9].c_str());
            grasp.grasp_pose.pose.position.y    = std::atof(db[i][10].c_str());
            grasp.grasp_pose.pose.position.z    = std::atof(db[i][11].c_str());
            grasp.grasp_pose.pose.orientation.w = std::atof(db[i][12].c_str());
            grasp.grasp_pose.pose.orientation.x = std::atof(db[i][13].c_str());
            grasp.grasp_pose.pose.orientation.y = std::atof(db[i][14].c_str());
            grasp.grasp_pose.pose.orientation.z = std::atof(db[i][15].c_str());

            //Static transformation to /r_hand
            staticTransform(grasp.grasp_pose.pose);


            grasp.grasp_posture.joint_names.resize(5);
            grasp.grasp_posture.joint_names[0] = "left_f0_j1";
            grasp.grasp_posture.joint_names[1] = "left_f1_j1";
            grasp.grasp_posture.joint_names[2] = "left_f2_j1";
            grasp.grasp_posture.joint_names[3] = "left_f1_j0";
            grasp.grasp_posture.joint_names[4] = "left_f2_j0";
            grasp.grasp_posture.points.resize(1);
            grasp.grasp_posture.points[0].positions.resize(5);
            grasp.grasp_posture.points[0].positions[0] = std::atof(db[i][4].c_str());
            grasp.grasp_posture.points[0].positions[1] = std::atof(db[i][5].c_str());
            grasp.grasp_posture.points[0].positions[2] = std::atof(db[i][6].c_str());
            grasp.grasp_posture.points[0].positions[3] = std::atof(db[i][7].c_str());
            grasp.grasp_posture.points[0].positions[4] = std::atof(db[i][8].c_str());

            grasp.grasp_posture.points[0].time_from_start = ros::Duration(3.0);

            grasp.pre_grasp_posture.joint_names.resize(5);
            grasp.pre_grasp_posture.joint_names[0] = "left_f0_j1";
            grasp.pre_grasp_posture.joint_names[1] = "left_f1_j1";
            grasp.pre_grasp_posture.joint_names[2] = "left_f2_j1";
            grasp.pre_grasp_posture.joint_names[3] = "left_f1_j0";
            grasp.pre_grasp_posture.joint_names[4] = "left_f2_j0";
            grasp.pre_grasp_posture.points.resize(1);
            grasp.pre_grasp_posture.points[0].positions.resize(5);
            grasp.pre_grasp_posture.points[0].positions[0] = 0.0;
            grasp.pre_grasp_posture.points[0].positions[1] = 0.0;
            grasp.pre_grasp_posture.points[0].positions[2] = 0.0;
            grasp.pre_grasp_posture.points[0].positions[3] = 0.0;
            grasp.pre_grasp_posture.points[0].positions[4] = 0.0;

            grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(3.0);

            grasp.pre_grasp_approach.direction.vector.y        = 1.0;
            grasp.pre_grasp_approach.direction.header.frame_id = this->hand_name_;
            grasp.pre_grasp_approach.min_distance              = 0.05;
            grasp.pre_grasp_approach.desired_distance          = 0.1;

            grasp.post_grasp_retreat.direction.header.frame_id = "world";
            grasp.post_grasp_retreat.direction.vector.z        = 1.0;
            grasp.post_grasp_retreat.min_distance              = 0.05;
            grasp.post_grasp_retreat.desired_distance          = 0.1;
            object_template_map_[type].grasps.push_back(grasp);
        }
        for(int i=0;i<object_template_map_[3].grasps.size(); i++)
            ROS_INFO("Grasp id inside ot: %s",object_template_map_[3].grasps[i].id.c_str());

        // Initialize states and control values
        grasp_status_.status  = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_code_    = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_severity_= RobotStatusCodes::OK;
        ROS_INFO(" Done initializing grasp library! ");

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
