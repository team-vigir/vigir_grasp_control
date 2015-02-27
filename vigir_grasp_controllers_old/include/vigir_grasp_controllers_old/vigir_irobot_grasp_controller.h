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

#ifndef VIGIR_IROBOT_GRASP_CONTROLLER_OLD_H
#define VIGIR_IROBOT_GRASP_CONTROLLER_OLD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <boost/algorithm/string.hpp>
#include <vector>

#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <sensor_msgs/JointState.h>

#include <vigir_grasp_controllers_old/vigir_grasp_controller.h>
#include <handle_msgs/HandleSensors.h>
#include <handle_msgs/HandleControl.h>

#include "geometric_shapes/mesh_operations.h"
#include "shape_msgs/Mesh.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"


#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

namespace vigir_grasp_controllers_old{

  /**
    * This class defines the wrapper for the Grasp Controller Plugin
    */

//# grasp id, template id, hand, initial grasp type, finger joints (12), final grasp pose relative to template (x,y,z,qx,qy,qz,qw), pre-grasp pose relative to template (x,y,z,qx,qy,qz,qw)
//122, 5, right, spherical, finger poses:, 0.126317, 1.465132, 1.066402, 0.082420, 1.453882, 1.052665, -0.066694, 1.436149, 1.212370, -0.215962, 0.608948, 0.927948, final pose:, -16.885791, -134.801227, 187.491598, 0.152884, -0.658311, -0.713172, -0.186118, pre-grasp:, -17.015441, -134.795114, 204.423572, -0.365684, 0.706135, 0.599623, 0.090010
    typedef enum
    {
        VELOCITY    = 1,
        POSITION    = 2,
        CURRENT     = 3,
        VOLTAGE     = 4
    } IRobotControlModes;

    typedef struct VigirIRobotFingerPoses
    {
        float fingers[5];
        //float f1[3];
        //float f2[3];
        //float f3[3];

        // This probably is dangerous and depends on packed alignment
        //FingerPoses(){ memset(f0,0,12*sizeof(float));}

        //Safer to do it like this
        VigirIRobotFingerPoses(){
            memset(fingers,0,5*sizeof(float));
            //memset(f1,0,3*sizeof(float));
            //memset(f2,0,3*sizeof(float));
            //memset(f3,0,3*sizeof(float));
        }

    } VigirIRobotFingerPoses;

    typedef struct VigirIRobotGraspSpecification : public VigirGraspSpecification
    {
        VigirIRobotFingerPoses finger_poses;
    } VigirIRobotGraspSpecification;

#define NUM_IROBOT_FINGER_JOINTS  5


  //////////////////////////////////////////////////////////////////////////
  // Defines the IRobot class as a derived class of VigirGraspController plus
  // extra iRobot hand specific info
  //////////////////////////////////////////////////////////////////////////

  class VigirIRobotGraspController: virtual public VigirGraspController
  {
      public:

        VigirIRobotGraspController();
        ~VigirIRobotGraspController();
        void initializeIRobotGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    protected:

        // Hand specific implementations used by the grasp controller
        GraspQuality processHandTactileData();
        void ZeroHandJointCommands()         ;
        void processHandSensorData()         ;// protected by sensor_data and write_data mutex locks
        void processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3& template_com);
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
        void setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])   ;
        void setHandOpeningData(const double& grasp_fraction)    ;

        // Hand specific callback used for processing hardware joint and tactile data
        void handleSensorsCallback(const handle_msgs::HandleSensors::ConstPtr &hs_msg);
        void calibrateCallback(const sensor_msgs::JointStateConstPtr &cal_msg );

        // Hand specific callbacks used by the physical iRobot hand
        // TODO: Make Struct for tactile data

        // Hand specific data

        // Simulation data recieved from ROS interface
        handle_msgs::HandleSensorsConstPtr       last_joint_state_msg_;

        // Variables to store sensor data locally for thread safe operation
        handle_msgs::HandleSensors                   local_handle_msg_;

        sensor_msgs::JointState                      tactile_feedback_msg_;
        //std::vector<handle_msgs::Finger>             local_tactile_msg_;
        //std::vector<float>                           local_palm_tactile_msg_;
        std::vector<handle_msgs::Finger>             tactile_zeros_;
        std::vector<float>                           palm_tactile_zeros_;

        VigirIRobotFingerPoses                        finger_poses_;
        std::vector< VigirIRobotGraspSpecification>   potential_grasps_;
        std::vector< VigirIRobotFingerPoses>          initial_finger_poses_; // cylindrical,prismatic, spherical
        std::vector< VigirIRobotFingerPoses>          final_finger_poses_;   // cylindrical,prismatic, spherical
        VigirIRobotFingerPoses                        current_finger_poses_;
        VigirIRobotFingerPoses                        real_initial_finger_poses_;

        std::vector<handle_msgs::Finger>             filtered_tactile_msg_;
        //std::vector<handle_msgs::Finger>             filtered_tactile_data_;

        std::vector<float>                           filtered_palm_tactile_msg_;
        std::vector<int>                             zero_joint_values_;

        std::vector<std::string>                     joint_names_;     // use this to hold finger names instead of using the joint_commands_.names
        handle_msgs::HandleControl                   joint_commands_;
        handle_msgs::HandleControl                   old_joint_commands_;

        int                                          zero_velocity_counter_;

        // Status message variable
        std::vector<flor_ocs_msgs::OCSRobotStatus>   hand_status_;
        std::vector<int>                             hand_temperature_counter_;
        std::vector<int>                             tactile_timeout_counter_;
        int                                          publish_counter_;


    private:

        void loadIRobotGraspDatabase(std::string& file_name);
        int staticTransform(geometry_msgs::Pose& palm_pose);

        ros::Publisher  irobot_states_pub_;
        ros::Publisher  irobot_status_pub_;
        ros::Publisher  joint_commands_pub_;
        ros::Publisher  aco_pub_;                    ///< Attached Collision Object Publisher
        ros::Subscriber measured_joint_state_sub_;  ///< Finger joint states
        ros::Subscriber calibration_sub_;
        //ros::Subscriber hand_tactile_sub_;

        float  spread_conversion;
        float  rad_to_pos_conversion;
  };

}




#endif // VIGIR_IROBOT_GRASP_CONTROLLER_H
