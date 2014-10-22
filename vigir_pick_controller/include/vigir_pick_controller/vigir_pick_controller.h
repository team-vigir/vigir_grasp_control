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

#ifndef VIGIR_PICK_CONTROLLER_H__
#define VIGIR_PICK_CONTROLLER_H__

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/algorithm/string.hpp>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <atlas_msgs/ForceTorqueSensors.h>
#include <atlas_msgs/AtlasState.h>

#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include "flor_ocs_msgs/OCSRobotStatus.h"
#include "flor_ocs_msgs/OCSGhostControl.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include "flor_control_msgs/FlorControlMode.h"
#include <flor_planning_msgs/PlanRequest.h>
#include <flor_atlas_msgs/AtlasHandMass.h>

#include <moveit_msgs/Grasp.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <urdf/model.h>

#include <boost/thread.hpp>
#include <vector>
#include <tf/tf.h>

#include <trajectory_msgs/JointTrajectory.h>

#define FINGER_EFFORTS 4

namespace vigir_pick_controller {


typedef enum
{
    NO_GRASP_QUALITY            = 0,
    PALM_AND_ALL_FINGERS        ,  // palm contact with all fingers including thumb
    PALM_AND_THUMB_PLUS_ONE     ,
    PALM_AND_THUMB_PLUS_TWO     ,  // only relevant on Sandia (use ALL for iRobot)
    PALM_AND_NO_THUMB           ,  // all other fingers except thumb
    PALM_AND_NO_THUMB_LESS_ONE  ,  // no thumb and not all fingers
    NO_PALM_AND_ALL_FINGERS     ,  // No palm contact, but otherwise all fingers (including thumb) in contact
    NO_PALM_AND_NO_THUMB        ,  // all fingers except thumb
    NO_PALM_AND_THUMB_PLUS_ONE  ,
    NO_PALM_AND_THUMB_PLUS_TWO  ,  // only relevant on Sandia (use ALL for iRobot)
    NUM_GRASP_QUALITIES
} GraspQuality;

typedef enum
{
    GRASP_MODE_NONE     = 0,
    TEMPLATE_GRASP_MODE = 1,
    MANUAL_GRASP_MODE   = 2,
    NUM_GRASP_MODES

} GraspControlModes;

typedef enum
{
    GRASP_STATE_NONE  = 0, // unknown state
    GRASP_INIT        = 1,
    APPROACHING       = 2,
    SURROUNDING       = 3,
    GRASPING          = 4,
    MONITORING        = 5,
    OPENING           = 6,
    GRASP_ERROR       = 7,
    NUM_GRASP_STATES

} GraspControlStates;

typedef enum
{
    GRASP_CYLINDRICAL = 0,
    GRASP_PRISMATIC   = 1,
    GRASP_SPHERICAL   = 2,
    GRASP_NONE        = 3,
    NUM_GRASP_TYPES
} GraspTypes;


struct VigirObjectTemplate
{
    uint16_t                                id;
    std::string                             name;
    float                                   mass;
    geometry_msgs::Point                    com;
    geometry_msgs::Point                    b_max;
    geometry_msgs::Point                    b_min;
    shape_msgs::Mesh                        mesh;
    std::vector<moveit_msgs::Grasp>         grasps;
    std::vector<geometry_msgs::PoseStamped> stand_poses;

    VigirObjectTemplate() : id(0),
                            mass(0.0)
    {
        //com   = geometry_msgs::Point(0.0,0.0,0.0);
    }
};


  // This is the generic grasp controller
  class VigirPickController
  {
  public:

      VigirPickController();
      virtual ~VigirPickController();

      void controllerLoop();
      void initializeGraspController(ros::NodeHandle &nh, ros::NodeHandle &nhp);

      // Hand specific implementations
      virtual GraspQuality processHandTactileData()  = 0;
      virtual void processHandSensorData()           = 0;// protected by sensor_data and write_data mutex locks
      virtual void processHandMassData(const tf::Transform& hand_T_template, float& template_mass, tf::Vector3& template_com) = 0;
      virtual void updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type) = 0;

      virtual void setHandEfforts(const double& grasp_effort, const double finger_effort[]) = 0;


      // ROS message callbacks
     void modeCommanderCallback(const flor_grasp_msgs::GraspState::ConstPtr &mc_msg);

     /** called to change from no grasping, to manual to template grasping while (this->run_flag) */
     int  processSetGraspMode(const flor_grasp_msgs::GraspState& mode_command);

     /** called to update planner status */
     void  plannerStatusCallback(const flor_ocs_msgs::OCSRobotStatus& planner_status);

     void controllerModeCallback(const flor_control_msgs::FlorControlMode& controller_mode);

     /** This function is called whenever the template pose is updated by the world modeling code
      * Make sure the template info matches,and then do transform selected grasp to wrist pose in world frame.
      * assump template pose is given in world frame
      */
     void  templateUpdateCallback(const flor_grasp_msgs::TemplateSelection& template_pose);

     /** This function is called whenever the template needs to be stitched to the real object.
      * assump template pose is given in world frame
      */
     void  templateStitchCallback(const flor_grasp_msgs::TemplateSelection& template_pose);

     /** called to update the latest wrist pose */
     void  wristPoseCallback(const geometry_msgs::PoseStamped& mode_command);

     /** called to update the latest hand offset pose */
     void handOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

     /** called whenever a new template/grasp pair is selected.
      *  Reset grasping state machine, and wait for updated matching template data with new pose.
      */
     void  graspSelectionCallback(const flor_grasp_msgs::GraspSelection& grasp);

     /** Set the current planning group to "arm only" or "arm + torso"           */
     void  graspPlanningGroupCallback(const flor_ocs_msgs::OCSGhostControl& planning_group);

    /** Release any currently active grasp, and return to the NONE state after opening.
     *  This requires a new grasp selection to restart.
     */
     void  releaseGraspCallback(const flor_grasp_msgs::GraspSelection& grasp);

     //std::vector< std::vector<QString> > readTextDBFile(QString path);


     void updateGraspStatus(); // call to publish latest grasp data

     void updateHandMass(); // call to publish latest grasp data



  protected:

    /**
     * This function copies the latest sensor data, and signals the main grasping control inside a worker thread
     * to execute the state machine and joint control.  It is called by the derived class once all sensor data is processed
     * and we're ready to do the control calculations.
     */
     void updatedSensorData();

    /**
     * This function must be called whenever the grasp controller state or mode changes
     *  to publish the current state.  Will be used by the arm control code.
     *
     * Use UInt8 to store state
     *    lower 4 bits = state machine state (up to 15 states) (define enum values)
     *    upper 4 bits =
     *          0x00?? = Controller mode (0=Off, 1=Template Grasping, 2=Manual
     *          0x??00 = ???
     */
    void updateActiveState();

    //Redundant, was the same as UpdateTemplateCallback
    //int processTemplateUpdate(const flor_grasp_msgs::TemplateSelection& template_pose);

   inline int16_t  getGraspStatus() { return RobotStatusCodes::status(grasp_status_code_, grasp_status_severity_);}
   void            setGraspStatus(const RobotStatusCodes::StatusCode& status, const RobotStatusCodes::StatusLevel& severity);


   // Filename of the grasping library
   std::string filename;



   // Time of the last joint_command update
    ros::Time                               lastControllerUpdateTime_;


    // Variables to store sensor data received from ROS interface
    flor_grasp_msgs::GraspSelection         last_grasp_msg_;
    flor_grasp_msgs::TemplateSelection      last_template_msg_;
    flor_grasp_msgs::GraspState             last_mode_msg_;
    flor_ocs_msgs::OCSRobotStatus           last_planner_status_msg_;
    flor_control_msgs::FlorControlMode      last_controller_mode_msg_;
    geometry_msgs::PoseStamped              last_wrist_pose_msg_;
    geometry_msgs::PoseStamped              last_wrist_error;
    double                                  last_position_error;
    double                                  last_orientation_error;
    bool                                    update_error_calc;

    // Variables to store locally sensor data recieved from ROS interface
    geometry_msgs::WrenchStamped            local_force_torque_msg_;
    geometry_msgs::PoseStamped              local_wrist_pose_msg_;


    // Internal data set by the controller (specifies right or left hand)
    std::string                             hand_name_;       // l_hand or r_hand
    int                                     hand_id_;         // -1=left, 1=right
    std::string                             planning_group_;

    // Active data specified by graspSelection/templateSelection commands
    int16_t                                 grasp_id_;
    int16_t                                 template_id_;
    int16_t                                 template_type_;
    uint8_t                                 grasp_type_;
    bool                                    template_updated_;
    bool                                    stitch_updated_;
    bool                                    release_grasp_;
    bool                                    grasp_updated_;

    // Internal variables used by active controllers
    geometry_msgs::Pose                     final_wrist_pose_;
    geometry_msgs::Pose                     pregrasp_wrist_pose_;
    tf::Transform                           stitch_template_pose_;
    tf::Transform                           hand_offset_pose_;
    tf::Transform                           gp_T_hand_;
    tf::Transform                           hand_T_template_;
    tf::Transform                           hand_T_palm_;
    flor_atlas_msgs::AtlasHandMass          hand_mass_msg_;
    geometry_msgs::PoseStamped              com_;

    // Internal variables used to store grasp mappings
    GraspQuality                            grasp_quality_;


    //Grasp status message
    flor_ocs_msgs::OCSRobotStatus      grasp_status_;
    RobotStatusCodes::StatusCode       grasp_status_code_;      // Using RobotStatusCodes with severity
    RobotStatusCodes::StatusLevel      grasp_status_severity_;

    // Main control loop for worker thread
    boost::thread  *                   worker_thread_;

    boost::mutex                       write_data_mutex_;
    boost::mutex                       control_wait_mutex_;
    boost::mutex                       thread_running_mutex_;
    boost::condition_variable          data_ready_condition_;
    bool                               sensor_data_ready_;
    bool                               run_flag; // allows worker thread to continue processing
    bool                               update_joint_commands;


  private:
    ros::Publisher active_state_pub_ ;
    ros::Publisher wrist_target_pub_ ;
    ros::Publisher template_stitch_pose_pub_ ;
    ros::Publisher wrist_plan_pub_   ;
    ros::Publisher grasp_status_pub_ ;
    ros::Publisher hand_mass_pub_ ;

    ros::Subscriber mode_commander_sub_;        ///< Grasping control mode
    ros::Subscriber grasp_selection_sub_;       ///< Current template and grasp selection message
    ros::Subscriber release_grasp_sub_;         ///< Releasgrasp_joint_controller.e grasp and reset the initial finger positions
    ros::Subscriber template_selection_sub_;    ///< Current template pose update
    ros::Subscriber hand_offset_sub_;           ///< Current hand offset pose update
    ros::Subscriber template_stitch_sub_;       ///< Current template pose to be stitched

    ros::Subscriber force_torque_sub_;          ///< Force torque including wrists
    ros::Subscriber planner_status_sub_;        ///< Planner status (for reporting bundled error messages)
    ros::Subscriber controller_mode_sub_;       ///< Controller mode (verify we can control appendages)
    ros::Subscriber grasp_planning_group_sub_;


    // called from within main loop to invoke handle updating of sensor data
    void convertSensorData();


  };


}
#endif