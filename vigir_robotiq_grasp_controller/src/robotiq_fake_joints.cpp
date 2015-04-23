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
    ros::Time start_grasp_time = ros::Time::now();

    while((current_time - start_grasp_time).toSec() < fake_time)
    {
        fakeJoints.hand_joints_.header.stamp = ros::Time::now();
        robotiq_fake_pub.publish(fakeJoints.hand_joints_);

        current_time = ros::Time::now();

        ros::spinOnce();
        ros::Duration(5).sleep();
    }

    return 0;
}
