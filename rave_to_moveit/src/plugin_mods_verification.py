import rospy
from osu_grasp_msgs.srv import *
from openravepy import *

import atlas_and_ik

def check_grasp_distance_callback(req):
	print "Got the service request!!"
	gt = SimEnvLoading.gt
	rave_pose = req.hand_pose.pose.orientation
	rave.pose.extend(req.hand_pose.pose.position)

	rave_transform = matrixFromPose(rave_pose)
	gt.drawTransform(rave_transform, length=0.3)
	raw_input("Does that look kind of like a grasp?")

def init_distance_service():
	distance_service = rospy.Service('CheckGraspDistance1', CheckGraspDistance, check_grasp_distance_callback)

	return distance_service
