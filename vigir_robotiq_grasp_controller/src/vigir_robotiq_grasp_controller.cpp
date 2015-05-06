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

//#include <vigir_grasp_control/vigir_robotiq_grasp_controller/include/vigir_robotiq_grasp_controller/vigir_robotiq_grasp_controller.h>
#include <vigir_robotiq_grasp_controller/vigir_robotiq_grasp_controller.h>

namespace vigir_robotiq_grasp_controller{


    VigirRobotiqGraspController::VigirRobotiqGraspController()
      : VigirManipulationController() // explicitly initialize the base class
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
      // Initialize the generic manipulation controller components
      initializeManipulationController(nh,nhp);

      ROS_INFO("Initializing Robotic Grasp controller");

      //Getting Joint States

      ros::SubscribeOptions robotiqJointStatesSo =
      ros::SubscribeOptions::create<sensor_msgs::JointState>("/grasp_control/" + wrist_name_ + "/joint_states", 1, boost::bind(&VigirRobotiqGraspController::robotiqJointStates_Callback, this, _1),ros::VoidPtr(), nh.getCallbackQueue());
      robotiqJointStates_sub_ = nh.subscribe(robotiqJointStatesSo);

      //Initializing Trajectory action for fingers
      this->trajectory_action_.trajectory.joint_names.resize(hand_joint_names_.size());
      this->trajectory_action_.trajectory.points.resize(1);
      this->trajectory_action_.trajectory.points[0].positions.resize(hand_joint_names_.size());
      this->trajectory_action_.trajectory.points[0].time_from_start = ros::Duration(0.5);
      this->trajectory_action_.goal_time_tolerance                  = ros::Duration(5.0);

      ROS_INFO("Trajectory action initialized");

      for(int i = 0; i < hand_joint_names_.size(); i++){
          ROS_INFO("Joint %d: %s",i,hand_joint_names_[i].c_str());
          this->trajectory_action_.trajectory.joint_names[i] = hand_joint_names_[i];
      }

      //THIS ARE SPECIFIC FROM ROBOTIQ HAND
      this->trajectory_action_.trajectory.points[0].positions[0]  = 1.22;
      this->trajectory_action_.trajectory.points[0].positions[1]  = 0.0;
      this->trajectory_action_.trajectory.points[0].positions[2]  = 1.13;
      this->trajectory_action_.trajectory.points[0].positions[3]  = 1.13;

      ROS_INFO("Close joint positions initialized");

      this->trajectory_client_ = new  vigir_robotiq_grasp_controller::TrajectoryActionClient("/"+this->hand_side_+"_robotiq/"+this->hand_side_+"_hand_traj_controller/follow_joint_trajectory", true);
      while(!this->trajectory_client_->waitForServer(ros::Duration(5.0)))
         ROS_INFO("Waititing for %s TrajectoryActionServer", this->hand_side_.c_str());

      //Sending Initial finger postions, MAX joint limit (CLOSE) from URDF
      if(this->trajectory_client_->isServerConnected())
      {
          ROS_INFO("Sending trajectory action");
          this->trajectory_action_.trajectory.header.stamp = ros::Time::now();
          this->trajectory_client_->sendGoal(trajectory_action_,
                                       boost::bind(&VigirRobotiqGraspController::trajectoryDoneCb, this, _1, _2),
                                       boost::bind(&VigirRobotiqGraspController::trajectoryActiveCB, this),
                                       boost::bind(&VigirRobotiqGraspController::trajectoryFeedbackCB, this, _1));
      }
      else
      {
          ROS_ERROR("TrajectoryActionClient: Server not yet connected!");
      }

    }
    void VigirRobotiqGraspController::graspCommandCallback(const flor_grasp_msgs::GraspState &grasp)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);

        //THIS IS ROBOTIQ SPECIFIC
        switch(grasp.grasp_state.data){
        case flor_grasp_msgs::GraspState::GRASP_ID:
            this->trajectory_action_.trajectory = grasp.grasp.grasp_posture;
        break;
        case flor_grasp_msgs::GraspState::OPEN:
            this->trajectory_action_.trajectory.points[0].positions[0]  = 0.0;
            this->trajectory_action_.trajectory.points[0].positions[1]  = 0.0;  //This joint behaves differentlly, spreads, not used for close
            this->trajectory_action_.trajectory.points[0].positions[2]  = 0.0;
            this->trajectory_action_.trajectory.points[0].positions[3]  = 0.0;
        break;
        case flor_grasp_msgs::GraspState::CLOSE:
            this->trajectory_action_.trajectory.points[0].positions[0]  = 1.22;
            this->trajectory_action_.trajectory.points[0].positions[1]  = 0.0;  //This joint behaves differentlly, spreads, not used for close
            this->trajectory_action_.trajectory.points[0].positions[2]  = 1.13;
            this->trajectory_action_.trajectory.points[0].positions[3]  = 1.13;
        break;
        case flor_grasp_msgs::GraspState::PERCENTAGE:
            this->trajectory_action_.trajectory.points[0].positions[0]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0122+float(grasp.finger_effort[0].data)*0.0122;
            this->trajectory_action_.trajectory.points[0].positions[1]  = last_joint_state_msg_.position[1];//float(grasp.finger_effort[3].data)*0.0028;  //This joint behaves differentlly, spreads, not used for close
            this->trajectory_action_.trajectory.points[0].positions[2]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0113+float(grasp.finger_effort[1].data)*0.0113;
            this->trajectory_action_.trajectory.points[0].positions[3]  = float(grasp.grip.data > 100 ? 100 : grasp.grip.data)*0.0113+float(grasp.finger_effort[2].data)*0.0113;
        break;
        default:return;
        }

        //Create ROS trajectory and publish
        if(this->trajectory_client_->isServerConnected())
        {
            this->trajectory_action_.trajectory.header.stamp = ros::Time::now();
            this->trajectory_client_->sendGoal(trajectory_action_,
                                         boost::bind(&VigirRobotiqGraspController::trajectoryDoneCb, this, _1, _2),
                                         boost::bind(&VigirRobotiqGraspController::trajectoryActiveCB, this),
                                         boost::bind(&VigirRobotiqGraspController::trajectoryFeedbackCB, this, _1));
        }
        else
        {
            ROS_ERROR("TrajectoryActionClient: Server not connected!");
        }

        return;
    }

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

    /////// ---------------------------------- Hardware callbacks ---------------------------------------
    void VigirRobotiqGraspController::robotiqJointStates_Callback(const sensor_msgs::JointState::ConstPtr& js_msg)
    {
        boost::lock_guard<boost::mutex> guard(this->write_data_mutex_);
        last_joint_state_msg_ = *(js_msg);
    }


    vigir_manipulation_controller::GraspQuality VigirRobotiqGraspController::processHandTactileData()
    {
        bool thumb      = false,
             index      = false,
             pinky      = false,
             palm = false;

        flor_grasp_msgs::LinkState  link_state;
        flor_grasp_msgs::HandStatus hand_status;

        hand_status  = getHandStatus();

        link_state.name.resize(hand_status.link_states.name.size());
        link_state.tactile_array.resize(hand_status.link_states.tactile_array.size());

        for (unsigned i = 0; i < link_state.tactile_array.size(); ++i)
        {
          link_state.tactile_array[i].pressure.resize(1); //Setting only one pressure contact for OCS visualization
        }

        for(unsigned int link_idx=0; link_idx<hand_status.link_states.tactile_array.size(); ++link_idx)  //Cycles through links with tactile arrays
        {
            float average_filter = 0.0;
            unsigned int count      = 0;
            for(unsigned int array_idx=0; array_idx<hand_status.link_states.tactile_array[link_idx].pressure.size(); array_idx++){ //Cycles through the array in this link
                if(hand_status.link_states.tactile_array[link_idx].pressure[array_idx] > 0.1){
                    average_filter+=hand_status.link_states.tactile_array[link_idx].pressure[array_idx];
                    count++;
                }
            }
            link_state.name[link_idx] = hand_status.link_states.name[link_idx]; //Sets the name of this link
            if(count>0){
                link_state.tactile_array[link_idx].pressure[0] = average_filter/count;

                //This is only to set a basic grasp quality
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
                link_state.tactile_array[link_idx].pressure[0] = 0;

        }

        setLinkState(link_state);

        if (palm && index && pinky && thumb)
            return vigir_manipulation_controller::PALM_AND_ALL_FINGERS;
        else if (palm && index && pinky)
            return vigir_manipulation_controller::PALM_AND_NO_THUMB;
        else if (palm &&  (index || pinky))
            return vigir_manipulation_controller::PALM_AND_NO_THUMB_LESS_ONE;
        else if (thumb && index && pinky)
            return vigir_manipulation_controller::NO_PALM_AND_ALL_FINGERS;
        else if (thumb && (index || pinky))
            return vigir_manipulation_controller::NO_PALM_AND_THUMB_PLUS_TWO;
        else
            return vigir_manipulation_controller::NO_GRASP_QUALITY;
    }

} /// end namespace vigir_robotiq_grasp_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_robotiq_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  vigir_robotiq_grasp_controller::VigirRobotiqGraspController robotiq_controller;

  ROS_WARN(" Initialize the Robotiq hand grasp controller ...");
  robotiq_controller.initializeRobotiqGraspController(nh, nhp);

  ROS_WARN(" Start the ros spinner ...");
  ros::spin();
  return 0;
}
