// This is a dummy node which will publish fake iRobot hand joint states so we
// can test in simulation.  We don't need this node if we have a working Gazebo plugin.

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

class VTHandFakeJoints
{
    public:
    ros::NodeHandle n;
    ros::Publisher irobot_fake_sensor_data;
    ros::Subscriber grasp_controller_sub;

    sensor_msgs::JointState hand_joints_;
    void VTHandInit(std::string hand);

};

// Initialize hand values to 0
void VTHandFakeJoints::VTHandInit(std::string hand)
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
    hand_joints_.name[1] = hand + "_f1_j2";
    hand_joints_.name[1] = hand + "_f2_j2";
    hand_joints_.name[1] = hand + "_f0_j3";
    hand_joints_.name[1] = hand + "_f1_j3";
    hand_joints_.name[1] = hand + "_f2_j3";

}


int main(int argc, char** argv)
{
    // Create publisher
    ros::init(argc, argv, "robotiq_hand_fake_joints");

    //Right hand class
    VTHandFakeJoints fakeRight;
    fakeRight.VTHandInit("r");

    ros::Publisher vt_hand_fake_pub;
    vt_hand_fake_pub = fakeRight.n.advertise<sensor_msgs::JointState>("/joint_states",1);

    //Left hand class
    VTHandFakeJoints fakeLeft;
    fakeLeft.VTHandInit("l");


    while(ros::ok())
    {
        fakeRight.hand_joints_.header.stamp = ros::Time::now();
        vt_hand_fake_pub.publish(fakeRight.hand_joints_);

        fakeLeft.hand_joints_.header.stamp = ros::Time::now();
        vt_hand_fake_pub.publish(fakeLeft.hand_joints_);

        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }

    return 0;
}
