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

#ifndef VIGIR_HOOK_GRASP_CONTROLLER_H__
#define VIGIR_HOOK_GRASP_CONTROLLER_H__


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/algorithm/string.hpp>
#include <vector>

#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <sensor_msgs/JointState.h>

#include <vigir_grasp_controllers/vigir_grasp_controller.h>

#include "geometric_shapes/mesh_operations.h"
#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"


#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

namespace vigir_grasp_controller{

  /**
    * This class defines the wrapper for the Grasp Controller Plugin
    */

//# grasp id, template id, hand, initial grasp type, finger joints (12), final grasp pose relative to template (x,y,z,qx,qy,qz,qw), pre-grasp pose relative to template (x,y,z,qx,qy,qz,qw)
//122, 5, right, spherical, finger poses:, 0.126317, 1.465132, 1.066402, 0.082420, 1.453882, 1.052665, -0.066694, 1.436149, 1.212370, -0.215962, 0.608948, 0.927948, final pose:, -16.885791, -134.801227, 187.491598, 0.152884, -0.658311, -0.713172, -0.186118, pre-grasp:, -17.015441, -134.795114, 204.423572, -0.365684, 0.706135, 0.599623, 0.090010

    typedef struct VigirHookGraspSpecification : public VigirGraspSpecification
    {
    } VigirHookGraspSpecification;
  //////////////////////////////////////////////////////////////////////////
  // Defines the Hook class as a derived class of VigirGraspController plus
  // extra Hook hand specific info
  //////////////////////////////////////////////////////////////////////////

  class VigirHookGraspController: virtual public VigirGraspController
  {
      public:

        VigirHookGraspController();
        ~VigirHookGraspController();
        void initializeHookGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    protected:

        // Hand specific implementations used by the grasp controller
        GraspQuality processHandTactileData();
        void ZeroHandJointCommands()         ;
        void processHandSensorData()         ;// protected by sensor_data and write_data mutex locks
        void processHandMassData(const tf::Transform& hand_T_template, float& template_mass, tf::Vector3& template_com);
        void publishHandStates(bool update_joint_commands)      ;
        void setInitialFingerPoses(const uint8_t& grasp_type)   ;
        void setFinalFingerPoses(const uint8_t& grasp_type)     ;
        void setInitialJointToCurrentPositions()                ;
        void setFingerPosesToID(const uint16_t& requested_release_id, const uint16_t& grasp_template_id);
        void updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type);

        void setHandEfforts(const double& grasp_effort, const double finger_effort[]);

        void setHandNoneData( );
        void setHandApproachingData(const double& grasp_fraction);
        void setHandSurroundingData( )                     ;
        void setHandGraspingData(const double& grasp_fraction, const int8_t finger_effort[])   ;
        void setAttachingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data) ;
        void setDetachingObject( )                     ;
        void setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])   ;
        void setHandOpeningData(const double& grasp_fraction)    ;

        // Hand specific callbacks used by the physical hook hand
        //void syncFingerCallback(const hook_hand_msgs::RawFingerStateConstPtr, const hook_hand_msgs::RawFingerStateConstPtr, const hook_hand_msgs::RawFingerStateConstPtr, const hook_hand_msgs::RawFingerStateConstPtr);

        void finger_0_Callback(const sensor_msgs::JointStateConstPtr& finger_msg);

        std::vector< VigirHookGraspSpecification>   potential_grasps_;

        std::vector<std::string>                     joint_names_;     // use this to hold finger names instead of using the joint_commands_.names
        osrf_msgs::JointCommands                     joint_commands_;

    private:

        void loadHookGraspDatabase(std::string& file_name);
        int staticTransform(geometry_msgs::Pose& palm_pose);

        ros::Publisher  joint_commands_pub_;
        ros::Publisher  aco_pub_;                    ///< Attached Collision Object Publisher
        ros::Subscriber measured_joint_state_sub_;  ///< Finger joint states
        ros::Subscriber hand_tactile_sub_;

        ros::Subscriber finger_0_sub_;
        ros::Subscriber finger_1_sub_;
        ros::Subscriber finger_2_sub_;
        ros::Subscriber finger_3_sub_;
        //message_filters::TimeSynchronizer finger_sync_;

  };

}
#endif
