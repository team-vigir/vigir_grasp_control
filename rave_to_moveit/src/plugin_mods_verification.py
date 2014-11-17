import rospy
from osu_grasp_msgs.srv import *
from openravepy import *
from osu_grasp_msgs.msg import *

import atlas_and_ik

def init_distance_service():
	distance_service = rospy.Service('CheckGraspDistance1', CheckGraspDistance, check_grasp_distance_callback)

	return distance_service

# This wont work if you use the plugin as client. The main script is stalled while the plugin is eqvaluating.
#def check_grasp_distance_callback(req):
#	#rospy.logerr("Got the service request!!")
#	global distance_pub
#	global gt
#	quat = req.hand_pose.pose.orientation
#	trans = req.hand_pose.pose.position
#	rave_pose = [quat.w, quat.x, quat.y, quat.z]
#	rave_pose.extend([trans.x, trans.y, trans.z])
#	print rave_pose

#	rave_transform = matrixFromPose(rave_pose)
#	drawing = gt.drawTransform(rave_transform, length=1)
#	raw_input("Does that look kind of like a grasp?")

#	distance_pub.publish(CheckGraspDistanceResponse(True))

def init_distance_subscriber_publisher():
	distance_pub = rospy.Publisher('osu_grasp_plugin/dist_resp', CheckGraspDistanceResponse, queue_size=2)
	distance_sub = rospy.Subscriber('osu_grasp_plugin/check_grasp_distance', CheckGraspDistanceRequest, check_grasp_distance_callback)

	print "Check grasp distance topics online."
	return distance_sub, distance_pub

def limit_params_for_check_distance(params, gt):
	#params['approachrays'] = [params['approachrays'][0]]
	ray = params['approachrays'][0]
	drawing = gt.drawTransform(atlas_and_ik.get_transform_for_approach(ray[0:3], -ray[3:6], 0))
	transform = atlas_and_ik.get_transform_for_approach(ray[0:3], -ray[3:6], params['rolls'][0])
	print "Approach direction: ", -ray[3:6]
	self.raveio.publish_preplugin_grasp(transform)
