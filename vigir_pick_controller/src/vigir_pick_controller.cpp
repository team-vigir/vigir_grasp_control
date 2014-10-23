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

#include <math.h>
#include <ros/ros.h>

//using namespace std;
#include <iostream>
#include <boost/thread/locks.hpp>
#include <fstream>

#include <vigir_pick_controller/vigir_pick_controller.h>
#include <flor_ocs_msgs/OCSGhostControl.h>

#include <flor_control_msgs/FlorControlModeCommand.h>

//#include<trajectory.h>
#include<trajectory_msgs/JointTrajectory.h>


//#include <moveit_core/robot_state/joint_state_group.h>
//#include <
//#include <moveit_core/robot_state/joint_state_group.h>
//#include </opt/vigir/catkin_ws/src/moveit>
//#include </opt/vigir/catkin_ws/src/moveit/moveit_core/robot_state/include/moveit/robot_state/joint_state_group.h>



namespace vigir_pick_controller{


VigirPickController::VigirPickController():
    update_error_calc(false),
    hand_name_( "unknown"), hand_id_(0), grasp_type_(0),
    template_updated_(false),stitch_updated_(true),release_grasp_(false),grasp_updated_(false),
    worker_thread_(NULL),
    sensor_data_ready_(false), run_flag(true),
    update_joint_commands(false)
{

    // Initialize to unknown mode
    last_mode_msg_.grasp_state.data        = 0;
    last_mode_msg_.grip.data               = 0;
    last_mode_msg_.finger_effort.resize(FINGER_EFFORTS);
    last_mode_msg_.finger_effort[0].data   = 0;
    last_mode_msg_.finger_effort[1].data   = 0;
    last_mode_msg_.finger_effort[2].data   = 0;
    last_mode_msg_.finger_effort[3].data   = 0;
    last_grasp_msg_.grasp_id.data          = 0;
    last_grasp_msg_.template_id.data       = 0;
    last_grasp_msg_.template_type.data     = 0;

}

VigirPickController::~VigirPickController()
  {
  std::cout << "Shutting down the Grasp joint controller ..." << std::endl;
  this->run_flag = false; // allow thread to terminate
  this->control_wait_mutex_.unlock(); // unlock to allow exiting

  std::cout << "Waiting for Grasp calculations to complete before exiting ..." << std::endl;
  boost::lock_guard<boost::mutex> running_lock(this->thread_running_mutex_);


  std::cout << "Kill the worker thread ..." << std::endl;
  if(worker_thread_)
    delete worker_thread_;
  std::cout << "Done with VigirPickController termination!" << std::endl;
  }


void VigirPickController::initializeGraspController(ros::NodeHandle &nh, ros::NodeHandle &nhp)
  {
    ROS_INFO("Entering Initialize grasp controller");

    // which hand are we controlling
    if (!nhp.hasParam("hand"))
    {
        ROS_WARN(" Did not find HAND parameter - using right hand as default");
    }

    // which file are we reading
    if (!nhp.hasParam("filename"))
    {
        ROS_WARN(" Did not find FILENAME parameter - using \"/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/grasp_library.grasp\" as default");
    }

    nhp.param<std::string>("filename", this->filename_,  "/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/grasp_library.grasp");
    nhp.param<std::string>("ot_filename", this->ot_filename_,  "/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/grasp_templates.txt");
    nhp.param<std::string>("hand", this->hand_name_,"r_hand");

    ROS_INFO("Hand parameterrs received");

    this->hand_id_   = 1;
    this->hand_side_ = "right";
    if ("l_hand" == this->hand_name_){
        this->hand_id_   = -1;
        this->hand_side_ = "left";
    }

    XmlRpc::XmlRpcValue   gp_T_hand;
    XmlRpc::XmlRpcValue   hand_T_palm;

    ROS_INFO("Selecting L/R Transformations");

    if (hand_id_>0)
    {
        planning_group_ = "r_arm_with_torso_group";
        nh.getParam("/r_hand_tf/gp_T_hand", gp_T_hand);
        ROS_ASSERT(gp_T_hand.getType() == XmlRpc::XmlRpcValue::TypeArray);
        nh.getParam("/r_hand_tf/hand_T_palm", hand_T_palm);
        ROS_INFO("Transformations selected for right");

    }
    else
    {
        planning_group_ = "l_arm_with_torso_group";
        nh.getParam("/l_hand_tf/gp_T_hand", gp_T_hand);
        ROS_ASSERT(gp_T_hand.getType() == XmlRpc::XmlRpcValue::TypeArray);
        nh.getParam("/l_hand_tf/hand_T_palm", hand_T_palm);
        ROS_INFO("Transformations selected for left");
    }



    gp_T_hand_.setOrigin(tf::Vector3(static_cast<double>(gp_T_hand[0]),static_cast<double>(gp_T_hand[1]),static_cast<double>(gp_T_hand[2])));
    gp_T_hand_.setRotation(tf::Quaternion(static_cast<double>(gp_T_hand[3]),static_cast<double>(gp_T_hand[4]),static_cast<double>(gp_T_hand[5]),static_cast<double>(gp_T_hand[6])));

    ROS_INFO("Graspit Transformations set");

    hand_T_palm_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    hand_T_palm_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    ROS_INFO("Initialize grasp controller for the %s ...", this->hand_name_.c_str());

    // Not sure why this waiting is performed in the original joint control tutorial
    // Maybe related to setting transport to UDP and waiting before Publishers are online?
    ros::Time last_ros_time_;
    bool wait = true;
    while (wait)
    {
      last_ros_time_ = ros::Time::now();
      if (last_ros_time_.toSec() > 0)
        wait = false;
    }

    ROS_INFO("Setup communications for the %s grasp controller ... ", hand_name_.c_str());


     active_state_pub_         = nh.advertise<flor_grasp_msgs::GraspState>("active_state",         1, true);
     wrist_target_pub_         = nh.advertise<geometry_msgs::PoseStamped>("wrist_target",          1, true);
     template_stitch_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("template_stitch_pose",  1, true);
     wrist_plan_pub_           = nh.advertise<flor_planning_msgs::PlanRequest>("wrist_plan",       1, true);
     grasp_status_pub_         = nh.advertise<flor_ocs_msgs::OCSRobotStatus>("grasp_status",       1, true);
     hand_mass_pub_            = nh.advertise<flor_atlas_msgs::AtlasHandMass>("hand_mass",         1, true);

    // These publishers should be remapped in launch file
     mode_commander_sub_    = nh.subscribe("mode_command",       1, &VigirPickController::modeCommanderCallback,  this);
     release_grasp_sub_     = nh.subscribe("release_grasp",      1, &VigirPickController::releaseGraspCallback,   this);
     template_selection_sub_= nh.subscribe("template_selection", 1, &VigirPickController::templateUpdateCallback, this);
     hand_offset_sub_       = nh.subscribe("hand_offset",        1, &VigirPickController::handOffsetCallback,     this);
     template_stitch_sub_   = nh.subscribe("template_stitch",    1, &VigirPickController::templateStitchCallback, this);
     planner_status_sub_    = nh.subscribe("planner_status",     1, &VigirPickController::plannerStatusCallback,  this);
     controller_mode_sub_   = nh.subscribe("controller_mode",    1, &VigirPickController::controllerModeCallback, this);
     grasp_selection_sub_   = nh.subscribe("grasp_selection",    1, &VigirPickController::graspSelectionCallback, this);
     grasp_planning_group_sub_   = nh.subscribe("/flor/ocs/ghost_ui_state",    1, &VigirPickController::graspPlanningGroupCallback, this);

     //Stitch template to hand transformation initialization
     this->stitch_template_pose_.setIdentity();

     this->hand_offset_pose_.setIdentity();

     ROS_WARN("Create worker thread for controls calculations ...");
     //this->worker_thread_ = new boost::thread(boost::bind(&VigirPickController::controllerLoop, this));

  }

///////////////////////////////////////////////////////
// Class Callback functions for ros subscribers


int VigirPickController::processSetGraspMode(const flor_grasp_msgs::GraspState& mode_command)
{
    // Store the latest state command, and update at next calculation loop
    boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
    this->last_mode_msg_ = mode_command;
    this->release_grasp_ = false; // reset the release command
    ROS_INFO("Requesting grasp mode %d in %s", (this->last_mode_msg_.grasp_state.data & 0xF0)>>4, hand_name_.c_str());

    return 0;
}


void VigirPickController::graspPlanningGroupCallback(const flor_ocs_msgs::OCSGhostControl& planning_group)
{

    if (planning_group.planning_group[2] == 1)
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_with_torso_group";
        else
            planning_group_ = "l_arm_with_torso_group";
    }
    else if (planning_group.planning_group[2] == 0)
    {
        if (hand_id_>0)
            planning_group_ = "r_arm_group";
        else
            planning_group_ = "l_arm_group";
    }

}

void  VigirPickController::plannerStatusCallback(const flor_ocs_msgs::OCSRobotStatus& planner_status)
{
    // Store the latest planner status and controller status at next calculation loop
    boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
    this->last_planner_status_msg_ = planner_status;
    return;

}

void VigirPickController::controllerModeCallback(const flor_control_msgs::FlorControlMode& controller_mode)
{
    // Store the latest gravity (appendage) controller mode and update grasp controller status at next calculation loop
    boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
    this->last_controller_mode_msg_ = controller_mode;
    return;

}

void VigirPickController::templateStitchCallback(const flor_grasp_msgs::TemplateSelection& template_pose)
{
    ROS_INFO("Confidence : %d",template_pose.confidence.data);
    if(template_pose.confidence.data >= 0)
    {
        //Calculate static transform to stitch template to hand
        tf::Transform wTh;
        tf::Transform wTt;
        tf::Transform tTg;

        wTh.setRotation(tf::Quaternion(this->local_wrist_pose_msg_.pose.orientation.x,this->local_wrist_pose_msg_.pose.orientation.y,this->local_wrist_pose_msg_.pose.orientation.z,this->local_wrist_pose_msg_.pose.orientation.w));
        wTh.setOrigin(tf::Vector3(this->local_wrist_pose_msg_.pose.position.x,this->local_wrist_pose_msg_.pose.position.y,this->local_wrist_pose_msg_.pose.position.z));

        wTt.setRotation(tf::Quaternion(template_pose.pose.pose.orientation.x,template_pose.pose.pose.orientation.y,template_pose.pose.pose.orientation.z,template_pose.pose.pose.orientation.w));
        wTt.setOrigin(tf::Vector3(template_pose.pose.pose.position.x,template_pose.pose.pose.position.y,template_pose.pose.pose.position.z));

        tTg.setRotation(tf::Quaternion(this->final_wrist_pose_.orientation.x,this->final_wrist_pose_.orientation.y,this->final_wrist_pose_.orientation.z,this->final_wrist_pose_.orientation.w));
        tTg.setOrigin(tf::Vector3(this->final_wrist_pose_.position.x,this->final_wrist_pose_.position.y,this->final_wrist_pose_.position.z));

        this->stitch_template_pose_ = hand_T_palm_ * hand_offset_pose_.inverse() * hand_T_palm_.inverse() * tTg.inverse() * wTt.inverse() * wTh;
    }else
    {
        //Set static transform to identity unstitching the template
        this->stitch_template_pose_.setIdentity();
    }
    //Publish to OCS
    if (template_stitch_pose_pub_)
    {
        geometry_msgs::PoseStamped stitch_template_pose;
        stitch_template_pose.header.frame_id = template_pose.pose.header.frame_id;
        stitch_template_pose.header.seq++;
        stitch_template_pose.header.stamp = template_pose.pose.header.stamp;
        stitch_template_pose.pose.position.x = this->stitch_template_pose_.getOrigin().getX();
        stitch_template_pose.pose.position.y = this->stitch_template_pose_.getOrigin().getY();
        stitch_template_pose.pose.position.z = this->stitch_template_pose_.getOrigin().getZ();
        stitch_template_pose.pose.orientation.w = this->stitch_template_pose_.getRotation().getW();
        stitch_template_pose.pose.orientation.x = this->stitch_template_pose_.getRotation().getX();
        stitch_template_pose.pose.orientation.y = this->stitch_template_pose_.getRotation().getY();
        stitch_template_pose.pose.orientation.z = this->stitch_template_pose_.getRotation().getZ();
        template_stitch_pose_pub_.publish(stitch_template_pose);
    }
    else
        ROS_WARN("Invalid template stitch pose publisher");
}

void VigirPickController::handOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    this->hand_offset_pose_.setRotation(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    this->hand_offset_pose_.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z) );
}

void VigirPickController::templateUpdateCallback(const flor_grasp_msgs::TemplateSelection& template_pose)
{
    // Store the latest state command, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        this->last_template_msg_ = template_pose;

        // Check to see that this data matches what was requested
        if (int16_t(template_pose.template_id.data) == this->template_id_)
        {
            if (this->grasp_id_)
            {
                this->template_updated_ = true;
            }
            else
            {
                //ROS_INFO(" Ignoring template update until we specify a grasp ID");
                ROS_INFO("       Ignoring template pose %d update for %s since we don't have valid grasp %d spec yet!",
                                        template_pose.template_id.data, hand_name_.c_str(), this->template_id_);
                this->template_updated_ = true;
            }

        }
        else
        {
            this->template_updated_ = false;
        }
    }

    return;
}

/** called to update the latest wrist pose */
void  VigirPickController::wristPoseCallback(const geometry_msgs::PoseStamped& wrist_pose)
{
    // Store the latest wrist data, and update at next calculation loop
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        this->last_wrist_pose_msg_ = wrist_pose;
        this->update_error_calc = true; // flag need to calculate error during controls

    }

     return;

}

void VigirPickController::graspSelectionCallback(const flor_grasp_msgs::GraspSelection& grasp)
{

     return;
}

void VigirPickController::releaseGraspCallback(const flor_grasp_msgs::GraspSelection& grasp)
{

     return;
}


void VigirPickController::modeCommanderCallback(const flor_grasp_msgs::GraspState::ConstPtr &mc_msg)
    {
      if (mc_msg)
      {
          //ROS_WARN("modeCommandCallback - commanded mode %d", (mc_msg->grasp_state.data & 0xF0) >> 4);
          int rc = processSetGraspMode(*mc_msg);
          if (rc)
          {
              ROS_WARN("Error setting grasping mode %d: ", (mc_msg->grasp_state.data & 0xF0) >> 4);
          }
      }
    }

void VigirPickController::updatedSensorData()
{

  // Store the latest data in thread safe manner
  {
      sensor_data_ready_     = true; // assumed atomic assignment
  }

  data_ready_condition_.notify_one(); // notify worker thread that we have something new

}


/// This function is called from within the main controller loop after we have the locks
void VigirPickController::convertSensorData()
    {

        // Convert the data
        processHandSensorData(); // call derived class function to get the data stored locally for safe processing within worker thread

        this->local_wrist_pose_msg_  = this->last_wrist_pose_msg_;

    }

// assume this function is called within mutex block

void VigirPickController::setGraspStatus(const RobotStatusCodes::StatusCode &status, const RobotStatusCodes::StatusLevel &severity)
{
    if ((RobotStatusCodes::NO_ERROR == this->grasp_status_code_)  || (RobotStatusCodes::GRASP_CONTROLLER_OK == this->grasp_status_code_))
    {
        this->grasp_status_code_      = status;
        this->grasp_status_severity_  = severity;
    }
    else
    {
        uint16_t current_code;
        uint8_t  current_severity;
        RobotStatusCodes::codes(this->grasp_status_code_, current_code, current_severity);
        if (this->grasp_status_severity_ < severity)
        {
            ROS_DEBUG(" Overwriting grasp controller error code %d:%d with %d:%d", this->grasp_status_code_, this->grasp_status_severity_, status, severity);
            this->grasp_status_code_      = status;
            this->grasp_status_severity_  = severity;
            return;
        }
        //else
        //{
        //    ROS_DEBUG(" Ignore grasp controller error code %d:%d due to more severe existing code %d:%d",status, severity, this->grasp_status_code_,this->grasp_status_severity_);
        //}
    }
}


/**
 * This function must be called to publish the grasp state machine status.
 */
inline void VigirPickController::updateGraspStatus()
{

    uint16_t current_status = RobotStatusCodes::status(this->grasp_status_code_,this->grasp_status_severity_);
    if (this->grasp_status_code_ == RobotStatusCodes::NO_ERROR)
    {
        // Assign a meaningful message to robot_status for annunciator window
        this->grasp_status_code_ = RobotStatusCodes::GRASP_CONTROLLER_OK;
    }
    if (current_status != grasp_status_.status)
    {

        ROS_INFO("   Update Grasp Status %d:%d  for %s", this->grasp_status_code_,this->grasp_status_severity_, this->hand_name_.c_str());
        grasp_status_.stamp  = ros::Time::now();
        grasp_status_.status = current_status;

        if (grasp_status_pub_)
            grasp_status_pub_.publish(grasp_status_);
        else
            ROS_WARN("Invalid grasp status publisher");
    }
//        else
//        {
//            ROS_INFO("   Maintain Grasp Status %d:%d  for %s", this->grasp_status_code_,this->grasp_status_severity_, this->hand_name_.c_str());

//        }

}

inline void VigirPickController::updateHandMass()
{

    if (hand_mass_pub_)
        hand_mass_pub_.publish(this->hand_mass_msg_);
    else
        ROS_WARN("Invalid hand_mass_pub_");
}


void VigirPickController::controllerLoop()
{

    // local variables used within control loop
    int8_t    requested_mode;
    int16_t   requested_grasp_id, requested_template_id, requested_template_type;
    bool      template_updated     = false;
    bool      release_grasp        = false;
    uint8_t   manual_type          =   0;
    uint16_t  planner_status_code  = RobotStatusCodes::GRASP_CONTROLLER_OK;
    uint8_t   planner_status_level = RobotStatusCodes::OK;

    flor_grasp_msgs::TemplateSelection last_template_data;
    flor_control_msgs::FlorControlMode controller_mode;

   // prevent shutdown
   boost::lock_guard<boost::mutex> running_lock(this->thread_running_mutex_);
   //ROS_INFO("Start the worker thread with run_flag = %d", this->run_flag);
   while (this->run_flag) // continue running until shutdown
   {
       {
       // Wait until we have new data worth processing
       //  and convert the sensor data to internal format
           boost::unique_lock<boost::mutex> sensor_data_lock(this->write_data_mutex_);
           while (!(this->sensor_data_ready_))
           {
               // unlock until signal received, then lock again
               // http://www.boost.org/doc/libs/1_41_0/doc/html/thread/synchronization.html#thread.synchronization.condvar_ref
               this->data_ready_condition_.wait(sensor_data_lock);
           }
           //ROS_WARN("Control loop sensor_data_ready = %d",this->sensor_data_ready_);
           this->sensor_data_ready_ = false;

           //Here is where the requested mode is set
           requested_mode   = (this->last_mode_msg_.grasp_state.data & 0xF0)>>4;
           manual_type      = (this->last_mode_msg_.grasp_state.data & 0x0F);
           release_grasp    = this->release_grasp_;

           // Main controller mode
           controller_mode = this->last_controller_mode_msg_;

           // Get planner status
           RobotStatusCodes::codes(this->last_planner_status_msg_.status,planner_status_code, planner_status_level);

           if (requested_mode > MANUAL_GRASP_MODE)
           {
               ROS_ERROR(" Invalid mode requested = %d - reset to none!", requested_mode);
               requested_mode = GRASP_MODE_NONE;
           }


           // This was the last request that we have
           requested_grasp_id      = this->last_grasp_msg_.grasp_id.data;
           requested_template_id   = this->last_grasp_msg_.template_id.data;
           requested_template_type = this->last_grasp_msg_.template_type.data;
           last_template_data      = this->last_template_msg_;
           template_updated        = this->template_updated_;

           //ROS_WARN(" requested mode = %d current state=%d", requested_mode, this->active_state_);
           this->convertSensorData(); // this is protected by the scoped lock and calls derived class processHandSensorData()
       }
       // END OF DATA LOCK AND DATA PROCESSING

       // Reset the status codes
       grasp_status_code_     = RobotStatusCodes::GRASP_CONTROLLER_OK;
       grasp_status_severity_ = RobotStatusCodes::OK;


       uint8_t current_mode = requested_mode;
       float    template_mass        = 0.0;
       tf::Vector3 template_com       = tf::Vector3(0.0,0.0,0.0);
       tf::Transform hand_T_template;
       hand_T_template.setIdentity();

       //ROS_WARN(" current mode = %d current state=%d (0x%x) prior mode=%d state=%d", current_mode, current_state, this->active_state_, prior_mode, prior_state);



       // If entering manual grasp mode for the first time, set finger values?
       if (current_mode == MANUAL_GRASP_MODE  && !release_grasp)
       {
           // Manual mode
           template_updated  = false;

           // Don't care about planner status if not in template mode
           planner_status_code  = RobotStatusCodes::GRASP_CONTROLLER_OK;
           planner_status_level = RobotStatusCodes::OK;

           // TODO: need to remove manual type and have it accept any grasp including templates
           if (grasp_updated_ ) // && manual_type < GRASP_NONE
           {
               boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
               // Use CYLINDRICAL, SPHERICAL, PRISMATIC
               // otherwise use the prior terminal state from template grasp

               grasp_updated_ = false;
           }
       }

        ////////////////////// END MANUAL MODE BLOCK ////////////////////////////////////

       //Process tactile information
       grasp_quality_ = this->processHandTactileData();

       this->processHandMassData(hand_T_template, template_mass, template_com);

       // Publish any changes to the grasp status
       this->updateGraspStatus();
       this->updateHandMass();

   } // end of while run
}
/////////////////////////////////////////////////////////////////////////
// END MAIN CONTROL LOOP
/////////////////////////////////////////////////////////////////////////
// Helper functions

std::vector< std::vector <std::string> > VigirPickController::readCSVFile(std::string& file_name){
    std::ifstream file ( file_name.c_str() );
    std::vector< std::vector <std::string> > db;
    for (std::string line; std::getline(file, line); )
    {
        std::istringstream in(line);
        std::vector<std::string> tmp;
        std::string value;

        while(std::getline(in, value, ',')) {
            std::stringstream trimmer;
            trimmer << value;
            value.clear();
            trimmer >> value; //removes white spaces
            tmp.push_back(value);
        }
        db.push_back(tmp);
    }
//    for(int i=0; i<db.size();i++){
//        for(int j=0; j<db[i].size();j++)
//            std::cout << db[i][j] << ",";
//        std::cout << "\n";
//    }
    return db;
}


void VigirPickController::loadObjectTemplateDatabase(std::string& file_name)
{
    std::vector< std::vector <std::string> > db = readCSVFile(file_name);

    for(int i = 1; i < db.size(); i++) //STARTING FROM 1 SINCE FIRST LINE IS HEADER BEGINING WITH "#"
    {
        VigirObjectTemplate object_template;
        unsigned int type = std::atoi(db[i][0].c_str());

        geometry_msgs::Point b_max;
        geometry_msgs::Point b_min;
        b_min.x = std::atof(db[i][2].c_str());
        b_min.y = std::atof(db[i][3].c_str());
        b_min.z = std::atof(db[i][4].c_str());
        b_max.x = std::atof(db[i][5].c_str());
        b_max.y = std::atof(db[i][6].c_str());
        b_max.z = std::atof(db[i][7].c_str());

        geometry_msgs::Point com ;
        com.x = std::atof(db[i][8].c_str());
        com.y = std::atof(db[i][9].c_str());
        com.z = std::atof(db[i][10].c_str());

        double mass = std::atof(db[i][11].c_str());

        object_template.b_max = b_max;
        object_template.b_min = b_min;
        object_template.com   = com;
        object_template.mass  = mass;
        object_template.id    = i-1;
        object_template.type  = type;
        object_template_map_.insert(std::pair<unsigned int,VigirObjectTemplate>(type,object_template));
    }
}



} // end of vigir_pick_controllers namespace






//#include <ros/ros.h>
//#include <vigir_pick_controller/vigir_pick_controller.h>


//// MoveIt!
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/move_group_interface/move_group.h>
//#include <shape_tools/solid_primitive_dims.h>


//#include <iostream>
//#include <boost/thread/locks.hpp>
//#include <fstream>

//#include <flor_ocs_msgs/OCSGhostControl.h>

//#include <flor_control_msgs/FlorControlModeCommand.h>


//static const std::string ROBOT_DESCRIPTION="robot_description";

//void pick(moveit::planning_interface::MoveGroup &group)
//{
//  std::vector<moveit_msgs::Grasp> grasps;

//  geometry_msgs::PoseStamped p;
//  p.header.frame_id    = "world";
//  p.pose.position.x    =  0.24277830431;
//  p.pose.position.y    = -0.745004221203;
//  p.pose.position.z    =  1.26911157683;
//  p.pose.orientation.x = -0.446351722051;
//  p.pose.orientation.y =  0.563471242384;
//  p.pose.orientation.z =  0.449446130412;
//  p.pose.orientation.w =  0.530347504081;

//  moveit_msgs::Grasp g;
//  g.grasp_pose = p;

//  g.pre_grasp_approach.direction.vector.y = -1.0;
//  g.pre_grasp_approach.direction.header.frame_id = "r_hand";
//  g.pre_grasp_approach.min_distance = 0.1;
//  g.pre_grasp_approach.desired_distance = 0.2;

//  g.post_grasp_retreat.direction.header.frame_id = "world";
//  g.post_grasp_retreat.direction.vector.z = 1.0;
//  g.post_grasp_retreat.min_distance = 0.1;
//  g.post_grasp_retreat.desired_distance = 0.25;

//  g.pre_grasp_posture.joint_names.resize(5);
//  g.pre_grasp_posture.joint_names[0] = "right_f0_j1";
//  g.pre_grasp_posture.joint_names[1] = "right_f1_j1";
//  g.pre_grasp_posture.joint_names[2] = "right_f2_j1";
//  g.pre_grasp_posture.joint_names[3] = "right_f1_j0";
//  g.pre_grasp_posture.joint_names[4] = "right_f2_j0";
//  g.pre_grasp_posture.points.resize(1);
//  g.pre_grasp_posture.points[0].positions.resize(5);
//  g.pre_grasp_posture.points[0].positions[0] = 0.0;
//  g.pre_grasp_posture.points[0].positions[1] = 0.0;
//  g.pre_grasp_posture.points[0].positions[2] = 0.0;
//  g.pre_grasp_posture.points[0].positions[3] = 0.0;
//  g.pre_grasp_posture.points[0].positions[4] = 0.0;

//  g.pre_grasp_posture.points[0].time_from_start = ros::Duration(8.0);

//  g.grasp_posture.joint_names.resize(5);
//  g.grasp_posture.joint_names[0] = "right_f0_j1";
//  g.grasp_posture.joint_names[1] = "right_f1_j1";
//  g.grasp_posture.joint_names[2] = "right_f2_j1";
//  g.grasp_posture.joint_names[3] = "right_f1_j0";
//  g.grasp_posture.joint_names[4] = "right_f2_j0";
//  g.grasp_posture.points.resize(1);
//  g.grasp_posture.points[0].positions.resize(5);
//  g.grasp_posture.points[0].positions[0] = 0.2;
//  g.grasp_posture.points[0].positions[1] = 0.2;
//  g.grasp_posture.points[0].positions[2] = 0.2;
//  g.grasp_posture.points[0].positions[3] = 0.0;
//  g.grasp_posture.points[0].positions[4] = 0.0;

//  g.grasp_posture.points[0].time_from_start = ros::Duration(8.0);

//  g.allowed_touch_objects.resize(1);
//  g.allowed_touch_objects[0] = "part";


//  g.id = "KAL_grasp";

//  grasps.push_back(g);
//  group.setSupportSurfaceName("table");
//  group.pick("part", grasps);
//}

//void place(moveit::planning_interface::MoveGroup &group)
//{
//  std::vector<moveit_msgs::PlaceLocation> loc;

//  geometry_msgs::PoseStamped p;
//  p.header.frame_id    = "world";
//  p.pose.position.x    =  0.5;
//  p.pose.position.y    = -0.6;
//  p.pose.position.z    =  1.225;
//  p.pose.orientation.x =  0.0;
//  p.pose.orientation.y =  0.0;
//  p.pose.orientation.z =  0.0;
//  p.pose.orientation.w =  1.0;
//  moveit_msgs::PlaceLocation g;
//  g.place_pose = p;

//  g.post_place_retreat.direction.vector.y = 1.0;
//  g.post_place_retreat.direction.header.frame_id = "r_hand";
//  g.post_place_retreat.min_distance = 0.1;
//  g.post_place_retreat.desired_distance = 0.25;

//  g.pre_place_approach.direction.vector.z = -1.0;
//  g.pre_place_approach.direction.header.frame_id = "world";
//  g.pre_place_approach.min_distance = 0.1;
//  g.pre_place_approach.desired_distance = 0.2;


//  g.post_place_posture.joint_names.resize(5);
//  g.post_place_posture.joint_names[0] = "right_f0_j1";
//  g.post_place_posture.joint_names[1] = "right_f1_j1";
//  g.post_place_posture.joint_names[2] = "right_f2_j1";
//  g.post_place_posture.joint_names[3] = "right_f1_j0";
//  g.post_place_posture.joint_names[4] = "right_f2_j0";
//  g.post_place_posture.points.resize(1);
//  g.post_place_posture.points[0].positions.resize(5);
//  g.post_place_posture.points[0].positions[0] = 0;
//  g.post_place_posture.points[0].positions[1] = 0;
//  g.post_place_posture.points[0].positions[2] = 0;
//  g.post_place_posture.points[0].positions[3] = 0;
//  g.post_place_posture.points[0].positions[4] = 0;

//  g.post_place_posture.points[0].time_from_start = ros::Duration(8.0);

//  loc.push_back(g);
//  group.setSupportSurfaceName("table");


//  // add path constraints
//  moveit_msgs::Constraints constr;
//  constr.orientation_constraints.resize(1);
//  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
//  ocm.link_name = "r_hand";
//  ocm.header.frame_id = p.header.frame_id;
//  ocm.orientation.x = 0.0;
//  ocm.orientation.y = 0.0;
//  ocm.orientation.z = 0.0;
//  ocm.orientation.w = 1.0;
//  ocm.absolute_x_axis_tolerance = 0.2;
//  ocm.absolute_y_axis_tolerance = 0.2;
//  ocm.absolute_z_axis_tolerance = M_PI;
//  ocm.weight = 1.0;
//  //  group.setPathConstraints(constr);
//  //group.setPlannerId("RRTConnectkConfigDefault");

//  group.place("part", loc);
//}

//int main(int argc, char **argv)
//{
//  ros::init (argc, argv, "right_arm_pick_place");
//  ros::AsyncSpinner spinner(1);
//  spinner.start();

//  ros::NodeHandle nh;
//  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
//  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

//  ros::WallDuration(1.0).sleep();

//  moveit::planning_interface::MoveGroup group("r_arm_group");
//  group.setPlanningTime(15.0);

//  // wait a bit for ros things to initialize
//  ros::WallDuration(1.0).sleep();

//  pick(group);

//  ros::WallDuration(1.0).sleep();

//  place(group);

//  ros::waitForShutdown();
//  return 0;
//}
