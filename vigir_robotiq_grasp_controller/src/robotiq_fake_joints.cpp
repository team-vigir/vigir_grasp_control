/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
// This is a dummy node which will publish fake iRobot hand joint states so we
// can test in simulation.  We don't need this node if we have a working Gazebo plugin.

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

class RobotiqFakeJoints
{
    public:
    ros::NodeHandle n;
    ros::Publisher irobot_fake_sensor_data;
    ros::Subscriber grasp_controller_sub;

    sensor_msgs::JointState hand_joints_;
    void RobotiqInit(std::string hand);

};

// Initialize hand values to 0
void RobotiqFakeJoints::RobotiqInit(std::string hand)
{
    hand_joints_.name.resize(7);
    hand_joints_.position.resize(7);
    hand_joints_.velocity.resize(7);
    hand_joints_.effort.resize(7);

    for(int i = 0; i < 7; i++)
    {
        hand_joints_.position[i] = 0;
        hand_joints_.velocity[i] = 0;
        hand_joints_.effort[i] = 0;
    }

    hand_joints_.name[0] = hand + "_f2_j0";
    hand_joints_.name[1] = hand + "_f0_j2";
    hand_joints_.name[2] = hand + "_f1_j2";
    hand_joints_.name[3] = hand + "_f2_j2";
    hand_joints_.name[4] = hand + "_f0_j3";
    hand_joints_.name[5] = hand + "_f1_j3";
    hand_joints_.name[6] = hand + "_f2_j3";

}


int main(int argc, char** argv)
{
    // Create publisher
    ros::init(argc, argv, "robotiq_hand_fake_joints");

    ros::NodeHandle np("~");


    std::string hand_prefix = "left";
    int fake_time = 120;


    //Right hand class
    RobotiqFakeJoints fakeJoints;

    if (!np.getParam("hand_prefix", hand_prefix))
    {
      ROS_ERROR("Could not get /hand_prefix parameter");
    }
    if (!np.getParam("fake_time", fake_time))
    {
      ROS_ERROR("Could not get /fake_time parameter");
    }
    fakeJoints.RobotiqInit(hand_prefix);

    ros::Publisher robotiq_fake_pub;
    robotiq_fake_pub = fakeJoints.n.advertise<sensor_msgs::JointState>("/joint_states",1);

    ros::Time current_time     = ros::Time::now();
    ros::Time start_grasp_time = current_time;

    //int i=0;

    while(ros::ok())//(i < fake_time/5)
    {
        fakeJoints.hand_joints_.header.stamp = ros::Time::now();
        robotiq_fake_pub.publish(fakeJoints.hand_joints_);

        //current_time = ros::Time::now();

        ros::spinOnce();
        ros::Duration(5).sleep();
        //i++;
    }

    return 0;
}
