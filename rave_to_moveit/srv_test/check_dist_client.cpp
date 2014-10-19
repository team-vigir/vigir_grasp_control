#include "ros/ros.h"
#include "osu_grasp_msgs/CheckGraspDistance.h"
#include "geometry_msgs/PoseStamped.h"

//ToDo:
//	put the serviceClient as a member in a class.
//	Add in the wait for service code.

class testClass{
public: 
	testClass() {
		srv_cli = n.serviceClient<osu_grasp_msgs::CheckGraspDistance>("CheckGraspDistance_test");
	};

	void run_service(){
		osu_grasp_msgs::CheckGraspDistance srv;
		srv.request.hand_pose = geometry_msgs::PoseStamped();
		srv.request.acceptable_dist = 1;

		if (srv_cli.call(srv)){
			ROS_INFO("Call did not hang. Response: %i", srv.response.within_acceptable_dist);

		} else {
			ROS_ERROR("Service call failed!");
		}

	};

private:
	ros::NodeHandle n;
	ros::ServiceClient srv_cli;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "srv_test_client");
	ros::NodeHandle n;

	ROS_INFO("Running classified version of service_test");	
	testClass cli;
	cli.run_service();
	return 0;
}
