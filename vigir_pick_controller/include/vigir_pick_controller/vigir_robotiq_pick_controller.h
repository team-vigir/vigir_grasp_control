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

#ifndef VIGIR_ROBOTIQ_GRASP_CONTROLLER_H__
#define VIGIR_ROBOTIQ_GRASP_CONTROLLER_H__


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/algorithm/string.hpp>
#include <vector>

#include <atlas_msgs/ForceTorqueSensors.h>
#include <sensor_msgs/JointState.h>

#include <vigir_pick_controller/vigir_pick_controller.h>

#include <moveit_msgs/Grasp.h>

//#define NUM_ROBOTIQ_FINGER_JOINTS  11
#define NUM_ROBOTIQ_PALM_JOINTS    4

namespace vigir_pick_controller{

  /**
    * This class defines the wrapper for the Grasp Controller Plugin
    */


  //////////////////////////////////////////////////////////////////////////
  // Defines the Robotiq class as a derived class of VigirPickController plus
  // extra Robotiq hand specific info
  //////////////////////////////////////////////////////////////////////////

  class VigirRobotiqPickController: virtual public VigirPickController
  {
      public:

        VigirRobotiqPickController();
        ~VigirRobotiqPickController();
        void initializeRobotiqPickController(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    protected:

        // Hand specific implementations used by the grasp controller
        GraspQuality processHandTactileData();
        void ZeroHandJointCommands()         ;
        void processHandSensorData()         ;// protected by sensor_data and write_data mutex locks
        void processHandMassData(const tf::Transform& hand_T_template, float& template_mass, tf::Vector3& template_com);
        void updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type);

        void setHandEfforts(const double& grasp_effort, const double finger_effort[]);


        //void tactileCallback(const sandia_hand_msgs::RawTactile::ConstPtr &tac_msg);

        std::map<unsigned int,VigirObjectTemplate>  template_map_;




    private:

        void loadRobotiqPickDatabase(std::string& file_name);
        void initTemplateIdMap(std::string& file_name);
        int staticTransform(geometry_msgs::Pose& palm_pose);

        ros::Subscriber hand_tactile_sub_;

        ros::Subscriber robotiqJointStates_sub_;

  };

}
#endif
