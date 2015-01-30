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

#include <vigir_grasp_controllers_old/vigir_robotiq_grasp_controller.h>

#define RAD_TO_BYTE    209.01638145
#define RAD_BC_TO_BYTE 225.663693848
#define SPREAD_RAD     0.28   //Radians range of the spread fingers
#define BYTE_TO_SPR    (SPREAD_RAD/255.0)
#define SPR_TO_BYTE    (1.0/BYTE_TO_SPR)
#define SPR_OFFSET     (BYTE_TO_SPR * 137.0)
#define SPR_OPEN       (-SPR_OFFSET)
#define SPR_CLOSE      SPREAD_RAD-SPR_OFFSET
#define PER_TO_BYTE    2.55

namespace vigir_grasp_controllers_old{


    VigirRobotiqGraspController::VigirRobotiqGraspController()
      : VigirGraspController() // explicitly initialize the base class
    {
    }

    VigirRobotiqGraspController::~VigirRobotiqGraspController()
    {
        std::cout << "Shutting down the Robotiq Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // Robotiq class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirRobotiqGraspController::initializeRobotiqGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {
      ROS_INFO("Entering initializeRobotiqGraspController for %s hand", hand_name_.c_str());
      // Initialize the generic grasp controller components
      initializeGraspController(nh,nhp);

      trajectory_client_ = new  TrajectoryActionClient("/"+this->hand_side_+"_robotiq/"+this->hand_side_+"_hand_traj_controller/follow_joint_trajectory", true);
      while(!trajectory_client_->waitForServer(ros::Duration(5.0)))
         ROS_INFO("Waititing for %s TrajectoryActionServer", this->hand_side_.c_str());

      ROS_WARN("TrajectoryActionServer Active...");

      ros::SubscribeOptions robotiqJointStatesSo =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_control/" + hand_name_ + "/joint_states", 1, boost::bind(&VigirRobotiqGraspController::robotiqJointStates_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      robotiqJointStates_sub_ = nh.subscribe(robotiqJointStatesSo);

      hand_status_sub_ = nh.subscribe("/grasp_control/" + hand_name_ + "/hand_status",       1, &VigirRobotiqGraspController::handStatusCallback,  this);
      tactile_feedback_pub_ = nh.advertise<flor_grasp_msgs::LinkState>("robotiq_states", 1, true);      

      aco_pub_                  = nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 1, true);
      co_pub_                   = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1, true);

      // Load the hand specific grasping database
      initializeEigenGrasps();

      this->link_tactile_.name.resize(NUM_ROBOTIQ_PALM_JOINTS);
      this->link_tactile_.tactile_array.resize(NUM_ROBOTIQ_PALM_JOINTS);

      for (unsigned i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
      {
        this->joint_commands_.rSP[i] = 255;
        this->joint_commands_.rFR[i] = 0.0;

        this->last_joint_commands_.rPR[i]= 0.0;
        this->last_joint_commands_.rFR[i]= 0.0;
        this->last_joint_commands_.rSP[i]= 0.0;
        this->link_tactile_.tactile_array[i].pressure.resize(1); //Setting only one pressure contact for OCS visualization
      }


    }


    // Hand specific implementations used by the grasp controller

    //
    void VigirRobotiqGraspController::ZeroHandJointCommands()
    {
        ROS_INFO("Setting ZeroHandJointCommands");
        for (unsigned i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
          this->joint_commands_.rPR[i] = 0.0;
          this->joint_commands_.rSP[i] = 255;
          this->joint_commands_.rFR[i] = 0.0;
        }
        return;

    }

    void VigirRobotiqGraspController::processHandSensorData()          // protected by sensor_data and write_data mutex locks
    {
        // Store the ROS message data into local structures for thread safe access
        if (last_joint_state_msg_)
            this->local_joint_state_msg_ = *(this->last_joint_state_msg_);
        //else
        //    ROS_INFO("Hand Joint Data does not exist");

        if( last_hand_status_msg_)
            this->local_hand_status_msg_     = *(this->last_hand_status_msg_);
        //else
        //    ROS_INFO("Hand Tactile Data does not exist");

    }

    void VigirRobotiqGraspController::processHandMassData(const tf::Transform& hand_T_template, float &template_mass, tf::Vector3 &template_com)
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

    void VigirRobotiqGraspController::publishHandStates(bool update_joint_commands)
    {
       // call as needed to publish the updated joint commands
       if (update_joint_commands)
       {
           bool new_command = false;
           for(unsigned int i=0; i<joint_names_.size();i++){
               if(fabs(joint_commands_.rPR[i] - last_joint_commands_.rPR[i])>0.004){
                   last_joint_commands_.rPR[i] = joint_commands_.rPR[i];
                   new_command = true;
               }
           }
           if(new_command){
               //ROS_INFO("Creating TrajectoryActionGoal");
               control_msgs::FollowJointTrajectoryGoal trajectory_action;
               trajectory_action.goal_time_tolerance = ros::Duration(5.0);
               trajectory_action.trajectory.joint_names.resize(11);
               trajectory_action.trajectory.joint_names[0]  = this->hand_side_+"_f0_j1";
               trajectory_action.trajectory.joint_names[1]  = this->hand_side_+"_f1_j1";
               trajectory_action.trajectory.joint_names[2]  = this->hand_side_+"_f2_j1";
               trajectory_action.trajectory.joint_names[3]  = this->hand_side_+"_f1_j0";
               trajectory_action.trajectory.joint_names[4]  = this->hand_side_+"_f2_j0";
               trajectory_action.trajectory.joint_names[5]  = this->hand_side_+"_f0_j2";
               trajectory_action.trajectory.joint_names[6]  = this->hand_side_+"_f1_j2";
               trajectory_action.trajectory.joint_names[7]  = this->hand_side_+"_f2_j2";
               trajectory_action.trajectory.joint_names[8]  = this->hand_side_+"_f0_j3";
               trajectory_action.trajectory.joint_names[9]  = this->hand_side_+"_f1_j3";
               trajectory_action.trajectory.joint_names[10] = this->hand_side_+"_f2_j3";
               trajectory_action.trajectory.points.resize(1);
               trajectory_action.trajectory.points[0].positions.resize(11);
               trajectory_action.trajectory.points[0].positions[0]  = joint_commands_.rPR[0];
               trajectory_action.trajectory.points[0].positions[1]  = joint_commands_.rPR[1];
               trajectory_action.trajectory.points[0].positions[2]  = joint_commands_.rPR[2];
               trajectory_action.trajectory.points[0].positions[3]  = joint_commands_.rPR[3];
               trajectory_action.trajectory.points[0].positions[4]  = 0.0;
               trajectory_action.trajectory.points[0].positions[5]  = 0.0;
               trajectory_action.trajectory.points[0].positions[6]  = 0.0;
               trajectory_action.trajectory.points[0].positions[7]  = 0.0;
               trajectory_action.trajectory.points[0].positions[8]  = 0.0;
               trajectory_action.trajectory.points[0].positions[9]  = 0.0;
               trajectory_action.trajectory.points[0].positions[10] = 0.0;
               trajectory_action.trajectory.points[0].time_from_start = ros::Duration(0.05);

               //Create ROS trajectory and publish
               if(trajectory_client_->isServerConnected())
               {
                   trajectory_action.trajectory.header.stamp = ros::Time::now();
                   trajectory_client_->sendGoal(trajectory_action,
                                                        //function that inside updates if(hand_id_>0)
                                                        //this->setGraspStatus(RobotStatusCodes::GRASP_R_CLOSURE_FAILURE, RobotStatusCodes::WARNING);
                                                        boost::bind(&VigirRobotiqGraspController::trajectoryDoneCb, this, _1, _2),
                                                        boost::bind(&VigirRobotiqGraspController::trajectoryActiveCB, this),
                                                        boost::bind(&VigirRobotiqGraspController::trajectoryFeedbackCB, this, _1));
               }
               else
               {
                   ROS_ERROR("TrajectoryActionClient: Server not connected!");
               }
           }
       }
       tactile_feedback_pub_.publish(link_tactile_);
    }

    // action callbacks for step plan request
    void VigirRobotiqGraspController::trajectoryActiveCB()
    {
        //ROS_INFO("TrajectoryActionClient: Status changed to active.");
    }

    void VigirRobotiqGraspController::trajectoryFeedbackCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {
        ROS_INFO("TrajectoryActionClient: Feedback received.");// pos[0]= %f", feedback->actual.positions[0]);
    }

    void VigirRobotiqGraspController::trajectoryDoneCb(const actionlib::SimpleClientGoalState& state,
                                                      const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        ROS_INFO("Fingers Trajectory finished in state [%s]", state.toString().c_str());
    }

    void VigirRobotiqGraspController::setAttachingObject(const tf::Transform& hand_T_template, const flor_grasp_msgs::TemplateSelection& last_template_data){
        //Add collision object with template pose and bounding box

        ROS_INFO("Removing collision object :%s started",(boost::to_string(int16_t(last_template_data.template_id.data))).c_str());
        /* First, define the REMOVE object message*/
        moveit_msgs::CollisionObject remove_object;
        remove_object.id = boost::to_string(int16_t(last_template_data.template_id.data));
        remove_object.header.frame_id = "world";
        remove_object.operation = remove_object.REMOVE;

        ROS_INFO("Collision object :%s removed",remove_object.id.c_str());

        co_pub_.publish(remove_object);

        ROS_INFO("Attach template started... ");

        // Define the attached object message
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // We will use this message to add or
        // subtract the object from the world
        // and to attach the object to the robot
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = this->hand_name_;
        /* The header must contain a valid TF frame*/
        attached_object.object.header.frame_id = this->hand_name_;
        /* The id of the object */
        attached_object.object.id = boost::to_string(int16_t(last_template_data.template_id.data));

        /* A default pose */
        geometry_msgs::Pose pose;
        pose.position.x    = hand_T_template.getOrigin().getX();
        pose.position.y    = hand_T_template.getOrigin().getY();
        pose.position.z    = hand_T_template.getOrigin().getZ();
        pose.orientation.w = hand_T_template.getRotation().getW();
        pose.orientation.x = hand_T_template.getRotation().getX();
        pose.orientation.y = hand_T_template.getRotation().getY();
        pose.orientation.z = hand_T_template.getRotation().getZ();

        //    /* Define a box to be attached */
        //    shape_msgs::SolidPrimitive primitive;  //HARDCODING TO 2X4X36 Debris
        //    primitive.type = primitive.BOX;
        //    primitive.dimensions.resize(3);
        //    primitive.dimensions[0] = last_template_data.bounding_max.x * 2 ;
        //    primitive.dimensions[1] = last_template_data.bounding_max.y * 2 ;
        //    primitive.dimensions[2] = last_template_data.bounding_max.z * 2 ;



        std::string mesh_path = "package://vigir_template_library/object_templates/"+last_template_data.mesh_path + ".ply";
        ROS_INFO("1 mesh_path: %s", mesh_path.c_str());
        shapes::Mesh* shape = shapes::createMeshFromResource(mesh_path);
        shapes::ShapeMsg mesh_msg;

    //    shape->computeVertexNormals();
    //    for(unsigned int i=0; i<shape->vertex_count*3;i+=3){
    //      shape->vertices[i]   = shape->vertices[i]   + shape->vertex_normals[i]  *0.02;
    //      shape->vertices[i+1] = shape->vertices[i+1] + shape->vertex_normals[i+1]*0.02;
    //      shape->vertices[i+2] = shape->vertices[i+2] + shape->vertex_normals[i+2]*0.02;
    //    }

        shapes::constructMsgFromShape(shape,mesh_msg);
        mesh_ = boost::get<shape_msgs::Mesh>(mesh_msg);

        attached_object.object.meshes.push_back(mesh_);
        attached_object.object.mesh_poses.push_back(pose);
    //    attached_object.object.primitives.push_back(primitive);
    //    attached_object.object.primitive_poses.push_back(pose);

        // Note that attaching an object to the robot requires
        // the corresponding operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;
        attached_object.touch_links.push_back(this->hand_side_ + "_palm");
        attached_object.touch_links.push_back(this->hand_side_ + "_f0_link_0");
        attached_object.touch_links.push_back(this->hand_side_ + "_f0_link_1");
        attached_object.touch_links.push_back(this->hand_side_ + "_f0_link_2");
        attached_object.touch_links.push_back(this->hand_side_ + "_f0_link_3");
        attached_object.touch_links.push_back(this->hand_side_ + "_f1_link_0");
        attached_object.touch_links.push_back(this->hand_side_ + "_f1_link_1");
        attached_object.touch_links.push_back(this->hand_side_ + "_f1_link_2");
        attached_object.touch_links.push_back(this->hand_side_ + "_f1_link_3");
        attached_object.touch_links.push_back(this->hand_side_ + "_f2_link_0");
        attached_object.touch_links.push_back(this->hand_side_ + "_f2_link_1");
        attached_object.touch_links.push_back(this->hand_side_ + "_f2_link_2");
        attached_object.touch_links.push_back(this->hand_side_ + "_f2_link_3");

        ROS_INFO("Attaching the object to the %s wrist", this->hand_name_.c_str());
        aco_pub_.publish(attached_object);
    }

    void VigirRobotiqGraspController::setDetachingObject(const flor_grasp_msgs::TemplateSelection& last_template_data )
    {
        /* First, define the DETACH object message*/
        moveit_msgs::AttachedCollisionObject detach_object;
        detach_object.object.id = boost::to_string(int16_t(last_template_data.template_id.data));
        detach_object.link_name = this->hand_name_;
        detach_object.object.operation = detach_object.object.REMOVE;

        ROS_INFO("Dettaching the object from the %s wrist", this->hand_name_.c_str());
        aco_pub_.publish(detach_object);
    }

    void VigirRobotiqGraspController::setInitialFingerPoses(const uint8_t& grasp_type)
    {
        if (grasp_type < this->initial_finger_poses_.size())
        {
            //ROS_ERROR("Use grasp type=%d to set the standard finger poses = initial", grasp_type);
            this->finger_poses_  = this->initial_finger_poses_.at(grasp_type);
        }
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
    }

    void VigirRobotiqGraspController::setFinalFingerPoses(const uint8_t& grasp_type)
    {
        if (grasp_type < this->final_finger_poses_.size())
            this->finger_poses_  = this->final_finger_poses_.at(grasp_type);
        else
            ROS_ERROR("Invalid grasp type=%d to set the standard finger poses ", grasp_type);
    }

    void VigirRobotiqGraspController::setInitialJointToCurrentPositions()
    {
        for(unsigned int i = 0; i < this->local_joint_state_msg_.name.size(); ++i)
        {
            if(this->local_joint_state_msg_.name[i].compare(this->hand_side_+"_f0_j1") == 0)
                real_initial_finger_poses_.f0[0] = this->local_joint_state_msg_.position[i]; //stores real joint positions 0
            else if(this->local_joint_state_msg_.name[i].compare(this->hand_side_+"_f1_j1") == 0)
                real_initial_finger_poses_.f0[1] = this->local_joint_state_msg_.position[i]; //stores real joint positions 1
            else if(this->local_joint_state_msg_.name[i].compare(this->hand_side_+"_f2_j1") == 0)
                real_initial_finger_poses_.f0[2] = this->local_joint_state_msg_.position[i]; //stores real joint positions 2
            else if(this->local_joint_state_msg_.name[i].compare(this->hand_side_+"_f1_j0") == 0)
                real_initial_finger_poses_.f0[3] = this->local_joint_state_msg_.position[i]; //stores real joint positions 3
        }
    }

    void VigirRobotiqGraspController::setFingerPosesToID(const uint16_t& requested_release_id, const uint16_t& grasp_template_id)
    {// Assumes data is locked by calling thread
//        for (std::vector <VigirRobotiqGraspSpecification>::iterator it = potential_grasps_.begin() ; it != potential_grasps_.end(); ++it)
//        {
//            if((*it).grasp_id == requested_release_id)
//            {
//                this->grasp_id_       = (*it).grasp_id;
//                this->template_id_    = grasp_template_id;
//                this->template_type_  = (*it).template_type;
//                this->finger_poses_   = (*it).finger_poses;   // assumes defined array fully copied
//            }
//        }

        if ( (this->grasp_id_ != requested_release_id))
        {
            ROS_WARN("%s: Release grasp selection id: %d not found.",
                     hand_name_.c_str(), requested_release_id);
        }

    }

    void VigirRobotiqGraspController::updateGraspTemplate(const uint16_t& requested_grasp_id, const uint16_t& requested_template_id, const uint16_t& requested_template_type)
    {
        for(int index = 0; index < last_template_res_.template_type_information.grasps.size(); index++)
        {
            if(std::atoi(last_template_res_.template_type_information.grasps[index].id.c_str()) == requested_grasp_id){
                boost::lock_guard<boost::mutex> sensor_data_lock(this->write_data_mutex_);
                this->grasp_id_            = requested_grasp_id;
                this->template_type_       = requested_template_type;
                this->template_id_         = requested_template_id;
                this->grasp_type_          = GRASP_CYLINDRICAL;

                this->finger_poses_.f0[0]  = last_template_res_.template_type_information.grasps[index].grasp_posture.points[0].positions[0];
                this->finger_poses_.f0[1]  = last_template_res_.template_type_information.grasps[index].grasp_posture.points[0].positions[4];
                this->finger_poses_.f0[2]  = last_template_res_.template_type_information.grasps[index].grasp_posture.points[0].positions[8];
                this->finger_poses_.f0[3]  = last_template_res_.template_type_information.grasps[index].grasp_posture.points[0].positions[3];

                this->final_wrist_pose_    = last_template_res_.template_type_information.grasps[index].grasp_pose.pose;
                this->pregrasp_wrist_pose_ = last_template_res_.template_type_information.grasps[index].grasp_pose.pose;
                staticTransform(this->final_wrist_pose_);

                gripperTranslationToPreGraspPose(this->pregrasp_wrist_pose_,last_template_res_.template_type_information.grasps[index].pre_grasp_approach);
                staticTransform(this->pregrasp_wrist_pose_);

                // Force a new round of template match to be sure and get wrist pose
                this->template_updated_    = false;

                break;
            }
        }

        if (-1 == this->grasp_id_ ) ROS_WARN("Failed to match grasp ID - set to -1");

    }

    void VigirRobotiqGraspController::setHandEfforts(const double& grasp_effort, const double finger_effort[])
    {
        //Effort settings in MANUAL mode
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
            this->joint_commands_.rFR[i] = grasp_effort+finger_effort[i]; //Index Proximal
        }
    }

    void VigirRobotiqGraspController::setHandNoneData( )
    {
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
            // Assumes that data is aligned, and we just move from buffer to next through f3[2]
            current_finger_poses_.f0[i]  = this->final_finger_poses_.at(GRASP_NONE).f0[i];
            this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];
            this->joint_commands_.rFR[i] = 0.0;
        }
    }

    void VigirRobotiqGraspController::setHandApproachingData(const double& grasp_fraction)
    {
        double one_minus_fraction = 1.0 - grasp_fraction;

        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.f0[i]  = real_initial_finger_poses_.f0[i] * one_minus_fraction +  //moving from the actual joint states avoids jumping from a full close to a full open, or between any two different grasps.
                                          initial_finger_poses_.at(this->grasp_type_).f0[i] * grasp_fraction;
           this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];
           this->joint_commands_.rFR[i] = 0.0;
        }

    }
    void VigirRobotiqGraspController::setHandSurroundingData( )
    {
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.f0[i]  = initial_finger_poses_.at(this->grasp_type_).f0[i]; // waste to recalc, but otherwise must keep track of transition
           this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];
           this->joint_commands_.rFR[i] = 0.0;
        }
    }

    void VigirRobotiqGraspController::setHandGraspingData(const double& grasp_fraction, const int8_t finger_effort[])
    {
        double one_minus_fraction = 1.0 - grasp_fraction;
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
           current_finger_poses_.f0[i] = initial_finger_poses_.at(this->grasp_type_).f0[i] * one_minus_fraction +
                                         this->finger_poses_.f0[i] * grasp_fraction + finger_effort[i]*0.01;

           //std::cout << "        f" << i << "  position=" << current_finger_poses_.f0[i] << " = " << initial_finger_poses_.at(this->grasp_type_).f0[i] << " + " << this->finger_poses_.f0[i]<< std::endl;
           this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];
           //this->joint_commands_.rFR[i] = finger_effort[i]; // @TODO - fix the scaling on this
        }
    }

    void VigirRobotiqGraspController::setHandMonitoringData(const double& grasp_effort, const int8_t finger_effort[])
    {
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
            // Assumes that data is aligned, and we just move from buffer to next through f3[2]
            //current_finger_poses_.f0[i]  = this->finger_poses_.f0[i]; // waste to recalc, but otherwise must keep track of transition
            //this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];

            this->joint_commands_.rFR[i]  = grasp_effort;
        }
    }

    void VigirRobotiqGraspController::setHandOpeningData(const double& grasp_fraction)
    {
        double one_minus_fraction = 1.0 - grasp_fraction;
        for(unsigned int i = 0; i < NUM_ROBOTIQ_PALM_JOINTS; ++i)
        {
           // Assumes that data is aligned, and we just move from buffer to next through f3[2]
           current_finger_poses_.f0[i] = real_initial_finger_poses_.f0[i] * one_minus_fraction +
                                                initial_finger_poses_.at(this->grasp_type_).f0[i] * grasp_fraction;
           //ROS_INFO("Setting finger %d to:  %f", i,current_finger_poses_.f0[i]);
           this->joint_commands_.rPR[i] = current_finger_poses_.f0[i];
           this->joint_commands_.rFR[i] = 0.0;
        }
    }

    void VigirRobotiqGraspController::handStatusCallback(const flor_grasp_msgs::HandStatus::ConstPtr &msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_hand_status_msg_ = msg;
    }

    /////// ---------------------------------- Hardware callbacks ---------------------------------------
    void VigirRobotiqGraspController::robotiqJointStates_Callback(const sensor_msgs::JointState::ConstPtr& js_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_joint_state_msg_ = js_msg;
        this->updatedSensorData();
    }
    /////// ---------------------------------- End Hardware callbacks ---------------------------------------

    // Hand specific messages for handling data
    bool FlorGraspSpecificationSorter(VigirRobotiqGraspSpecification const& a, VigirRobotiqGraspSpecification const& b)
    {
        if (a.grasp_id < b.grasp_id) return true;
        return false;
    }

    void VigirRobotiqGraspController::initializeEigenGrasps()
    {
        std::string tmp_hand_name;

        this->joint_names_.clear(); // reset the vector

        // must match those inside of the /robotiq_hands/?_hand/joint_states/[right_/left_]+
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
        this->joint_names_.push_back(tmp_hand_name+"sprd");

        if (NUM_ROBOTIQ_PALM_JOINTS != this->joint_names_.size())
        {   // should throw exception here and abort plugin loading
            ROS_ERROR("Invalid joint names - don't match our vectors %d vs. %d", uint8_t(this->joint_names_.size()), NUM_ROBOTIQ_PALM_JOINTS);
        }
        ROS_INFO(" Resizing finger poses vectors for grasp types ...");

        initial_finger_poses_.resize(NUM_GRASP_TYPES);
        final_finger_poses_.resize(NUM_GRASP_TYPES);

        ROS_INFO(" Setup initial finger poses for grasp types ...");

        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[0] =0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[1] =0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[2] =0.0;
        initial_finger_poses_.at(GRASP_CYLINDRICAL).f0[3] =0.0;

        initial_finger_poses_.at(GRASP_PRISMATIC).f0[0] =0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).f0[1] =0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).f0[2] =0.0;
        initial_finger_poses_.at(GRASP_PRISMATIC).f0[3] =SPR_CLOSE;

        initial_finger_poses_.at(GRASP_SPHERICAL).f0[0] =0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).f0[1] =0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).f0[2] =0.0;
        initial_finger_poses_.at(GRASP_SPHERICAL).f0[3] =SPR_OPEN;

        initial_finger_poses_.at(GRASP_NONE).f0[0] =0.0;
        initial_finger_poses_.at(GRASP_NONE).f0[1] =0.0;
        initial_finger_poses_.at(GRASP_NONE).f0[2] =0.0;
        initial_finger_poses_.at(GRASP_NONE).f0[3] =0.0;

        ROS_INFO(" Setup final finger poses for grasp types ...");

        // TODO - define terminal finger positions with fully closed manual grasp
        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[0] =1.22;
        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[1] =1.13;
        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[2] =1.13;
        final_finger_poses_.at(GRASP_CYLINDRICAL).f0[3] =0.0;

        final_finger_poses_.at(GRASP_PRISMATIC).f0[0] =0.53;
        final_finger_poses_.at(GRASP_PRISMATIC).f0[1] =0.49;
        final_finger_poses_.at(GRASP_PRISMATIC).f0[2] =0.49;
        final_finger_poses_.at(GRASP_PRISMATIC).f0[3] =SPR_CLOSE;

        final_finger_poses_.at(GRASP_SPHERICAL).f0[0] =1.22;
        final_finger_poses_.at(GRASP_SPHERICAL).f0[1] =1.13;
        final_finger_poses_.at(GRASP_SPHERICAL).f0[2] =1.13;
        final_finger_poses_.at(GRASP_SPHERICAL).f0[3] =SPR_OPEN;

        final_finger_poses_.at(GRASP_NONE).f0[0] =1.22;
        final_finger_poses_.at(GRASP_NONE).f0[1] =1.13;
        final_finger_poses_.at(GRASP_NONE).f0[2] =1.13;
        final_finger_poses_.at(GRASP_NONE).f0[3] =0.0;

        ROS_INFO(" Done setting intital and final poses");

        active_state_.grasp_state.data = (TEMPLATE_GRASP_MODE << 4) + GRASP_STATE_NONE; // Nothing is specified until we get a grasp command AND template pose
        grasp_id_       = -1 ;
        template_id_    = -1 ;
        template_type_  = -1 ;
        grasp_type_     = GRASP_CYLINDRICAL;
        start_grasp_flag = false;
        grasp_period     = 3.0;    // in seconds
        grasp_gain       = 10.0;
        finger_open_threshold  = 0.99;
        finger_close_threshold = 0.95;
        grasp_status_.status  = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_code_    = RobotStatusCodes::GRASP_CONTROLLER_OK;
        grasp_status_severity_= RobotStatusCodes::OK;

        ROS_INFO(" Done initializing states and control values");

    }



    GraspQuality VigirRobotiqGraspController::processHandTactileData()
    {

        bool thumb      = false,
             index      = false,
             pinky      = false,
             palm = false;

        for(unsigned int link_idx=0; link_idx<local_hand_status_msg_.link_states.tactile_array.size(); ++link_idx)  //Cycles through links with tactile arrays
        {
            float average_filter = 0.0;
            unsigned int count      = 0;
            for(unsigned int array_idx=0; array_idx<local_hand_status_msg_.link_states.tactile_array[link_idx].pressure.size(); array_idx++){
                if(local_hand_status_msg_.link_states.tactile_array[link_idx].pressure[array_idx] > 0.1){
                    average_filter+=local_hand_status_msg_.link_states.tactile_array[link_idx].pressure[array_idx];
                    count++;
                }
            }
            this->link_tactile_.name[link_idx] = local_hand_status_msg_.link_states.name[link_idx];
            if(count>0){
                this->link_tactile_.tactile_array[link_idx].pressure[0] = average_filter/count;
                switch (link_idx) {
                case 0:
                    thumb = true;
                    break;
                case 1:
                    index = true;
                    break;
                case 2:
                    pinky = true;
                    break;
                case 3:
                    palm = true;
                    break;
                default:
                    break;
                }
            }else
                this->link_tactile_.tactile_array[link_idx].pressure[0] = 0;

        }

        if (palm && index && pinky && thumb)
            return PALM_AND_ALL_FINGERS;
        else if (palm && index && pinky)
            return PALM_AND_NO_THUMB;
        else if (palm &&  (index || pinky))
            return PALM_AND_NO_THUMB_LESS_ONE;
        else if (thumb && index && pinky)
            return NO_PALM_AND_ALL_FINGERS;
        else if (thumb && (index || pinky))
            return NO_PALM_AND_THUMB_PLUS_TWO;
        else
            return NO_GRASP_QUALITY;
    }

    // transform endeffort to palm pose used by GraspIt
    int VigirRobotiqGraspController::staticTransform(geometry_msgs::Pose& palm_pose)
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
