#!/usr/bin/env python
import rospy
from osu_grasp_msgs.srv import *

def check_grasp_distance_callback(lol):
	print "Service called!"
	return CheckGraspDistanceResponse(True)

def start_service():
	serv = rospy.Service('CheckGraspDistance_test', CheckGraspDistance, check_grasp_distance_callback)
	return serv

if __name__ == "__main__":
	rospy.init_node("service_test")
	serv = start_service()
	rospy.spin()
