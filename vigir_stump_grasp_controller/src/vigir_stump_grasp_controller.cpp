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

//#include <vigir_grasp_control/vigir_stump_grasp_controller/include/vigir_stump_grasp_controller/vigir_stump_grasp_controller.h>
#include <vigir_stump_grasp_controller/vigir_stump_grasp_controller.h>

namespace vigir_stump_grasp_controller{


    VigirStumpGraspController::VigirStumpGraspController()
      : VigirManipulationController() // explicitly initialize the base class
    {
    }

    VigirStumpGraspController::~VigirStumpGraspController()
    {
        std::cout << "Shutting down the Stump Hand grasping controller ..." << std::endl;
    }


    //////////////////////////////////////////////////////////////////////////
    // Stump class functions
    //////////////////////////////////////////////////////////////////////////

    void VigirStumpGraspController::initializeStumpGraspController(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    {
      // Initialize the generic manipulation controller components
      initializeManipulationController(nh,nhp);

    }
    void VigirStumpGraspController::graspCommandCallback(const vigir_grasp_msgs::GraspState &grasp)
    {
    }

    vigir_manipulation_controller::GraspQuality VigirStumpGraspController::processHandTactileData()
    {
    }


} /// end namespace vigir_stump_grasp_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_stump_controller");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  vigir_stump_grasp_controller::VigirStumpGraspController stump_controller;

  ROS_WARN(" Initialize the stump hand grasp controller ...");
  stump_controller.initializeStumpGraspController(nh, nhp);

  ROS_WARN(" Start the ros spinner ...");
  ros::spin();
  return 0;
}
