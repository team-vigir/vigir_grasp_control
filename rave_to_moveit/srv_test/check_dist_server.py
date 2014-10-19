#!/usr/bin/env python

from osu_grasp_msgs.srv import *
import rospy

#Make some subscribers?

def handle_grasp_dist_check(req):
	print "Service called! Pose: ", req.hand_pose
	return CheckGraspDistanceResponse(True)

def check_grasp_dist_server():
	rospy.init_node('srv_test_service')
	s = rospy.Service('CheckGraspDistance_test', CheckGraspDistance, handle_grasp_dist_check)
	print "Server online. Returning service handle."
	return s

def new_funct():
	rospy.spin()

if __name__ == "__main__":
	s = check_grasp_dist_server()
	new_funct()
