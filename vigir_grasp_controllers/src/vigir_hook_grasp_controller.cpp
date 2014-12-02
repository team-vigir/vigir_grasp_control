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


#include <vigir_grasp_controllers/vigir_hook_grasp_controller.h>


namespace vigir_grasp_controller{


    VigirHookGraspController::VigirHookGraspController()
      : VigirGraspController() // explicitly initialize the base class
    {

    }

    VigirHookGraspController::~VigirHookGraspController()
    {
        std::cout << "Shutting down the Hook Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // Hook class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirHookGraspController::initializeHookGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {

      // Initialize the generic grasp controller components
      initializeGraspController(nh,nhp);

      // Load the hand specific grasping database
      loadHookGraspDatabase(this->filename);

    }


    // Hand specific implementations used by the grasp controller

    //
    void VigirHookGraspController::ZeroHandJointCommands()
    {
    }

    void VigirHookGraspController::processHandSensorData()          // protected by sensor_data and write_data mutex locks
    {
    }

    void VigirHookGraspController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
    {
        tf::Vector3 hand_com;
        if (hand_id_>0)
        {
            hand_com  = tf::Vector3(0.0,-0.20,0.0);
        }
        else
        {
            hand_com  = tf::Vector3(0.0,0.20,0.0);
        }
        float      hand_mass = 0.3;
        template_com = hand_T_template * template_com;
        hand_com     *= hand_mass;
        template_com *= template_mass;
        hand_com     += template_com;

        this->hand_mass_msg_.hand  = hand_id_ < 1 ? 0: 1;  //TO KEEP CONSISTENCY WITH BDI LEFT=0/RIGHT=1
        this->hand_mass_msg_.mass = hand_mass + template_mass;
        if(this->hand_mass_msg_.mass != 0)
            hand_com            /= this->hand_mass_msg_.mass;
        else{
            hand_com.setX(0.0);
            hand_com.setY(0.0);
            hand_com.setZ(0.0);
        }
        com_.header.frame_id = "/"+this->hand_name_;
        com_.pose.position.x = hand_com.getX();
        com_.pose.position.y = hand_com.getY();
        com_.pose.position.z = hand_com.getZ();
        this->hand_mass_msg_.com = com_;
    }

    void VigirHookGraspController::publishHandStates(bool update_joint_commands)
    {
    }

    void VigirHookGraspController::setInitialFingerPoses(const uint8_t& grasp_type)
    {
    }

    void VigirHookGraspController::setFinalFingerPoses(const uint8_t& grasp_type)
    {
    }

    void VigirHookGraspController::setInitialJointToCurrentPositions()
    {
    }

    void VigirHookGraspController::setFingerPosesToID(const uint16_t& requested_release_id, const uint16_t& grasp_template_id)
    {// Assumes data is locked by calling thread
        for (std::vector <VigirHookGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
        {
            if((*it).grasp_id == requested_release_id)
            {
                this->grasp_id_       = (*it).grasp_id;
                this->template_id_    = grasp_template_id;
                this->template_type_  = (*it).template_type;
            }
        }

        if ( (this->grasp_id_ != requested_release_id))
        {
            ROS_WARN("%s: Release grasp selection id: %d not found.",
                     hand_name_.c_str(), requested_release_id);
        }

    }

    void VigirHookGraspController::updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type)
    {
        for (std::vector <VigirHookGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
        {
           if((*it).grasp_id == requested_grasp_id && (*it).template_type == requested_template_type)
           {
               boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
               this->grasp_id_            = (*it).grasp_id;
               this->template_type_       = (*it).template_type;
               this->template_id_         = requested_template_id;
               this->final_wrist_pose_    = (*it).final_wrist_pose;
               this->pregrasp_wrist_pose_ = (*it).pregrasp_wrist_pose;
               this->grasp_type_          = (*it).grasp_type;

               // Force a new round of template match to be sure and get wrist pose
               this->template_updated_    = false;
           }

        }

        if (-1 == this->grasp_id_ ) ROS_WARN("Failed to match grasp ID - set to -1");

    }
    void VigirHookGraspController::setAttachingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data)
    {
    }

    void VigirHookGraspController::setDetachingObject( )
    {
    }

    void VigirHookGraspController::setHandEfforts(const double& grasp_effort, const double finger_effort[])
    {
    }

    void VigirHookGraspController::setHandNoneData( )
    {
    }

    void VigirHookGraspController::setHandApproachingData(const double& grasp_fraction)
    {
    }

    void VigirHookGraspController::setHandSurroundingData( )
    {
    }

    void VigirHookGraspController::setHandGraspingData(const double& grasp_fraction, const int8_t finger_effort[])
    {
    }

    void VigirHookGraspController::setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])
    {
    }

    void VigirHookGraspController::setHandOpeningData(const double& grasp_fraction)
    {
    }

    // Hand specific messages for handling data
    bool FlorGraspSpecificationSorter(VigirHookGraspSpecification const& a, VigirHookGraspSpecification const& b)
    {
        if (a.grasp_id < b.grasp_id) return true;
        return false;
    }

    void VigirHookGraspController::loadHookGraspDatabase(std::string& file_name)
    {
        /// @TODO - need to add expection protection for reading data out of range

        ROS_INFO("Setup default joint names for the %s hand", hand_name_.c_str());

        // Read the files to load all template and grasping data

        // TODO - need to make this a paramter that we can set by calling from plugin
        std::ifstream file ( file_name.c_str() );
        std::string grasp_line;
        VigirHookGraspSpecification grasp_spec;
        std::string tmp_hand_name;
        int8_t      tmp_hand_id;

        potential_grasps_.clear();
        potential_grasps_.push_back(VigirHookGraspSpecification()); // default 0 grasp

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



    GraspQuality VigirHookGraspController::processHandTactileData()
    {
        return NO_GRASP_QUALITY;
    }

    // transform endeffort to palm pose used by GraspIt
    int VigirHookGraspController::staticTransform(geometry_msgs::Pose& palm_pose)
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
