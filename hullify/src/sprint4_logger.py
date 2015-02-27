#!/usr/bin/env python
import rospy
from osu_grasp_msgs.msg import Mesh_and_bounds
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker
from takktile_ros.msg import Touch
from flor_grasp_msgs.msg import GraspSelection
from copy import deepcopy as copy

import tf
from tf import transformations

import os
import rospkg
rospack = rospkg.RosPack()
hullify_base_path = rospack.get_path("hullify")


class GraspStates:
	def __init_(self):
		self.grasp_states = []
	
	def add_state(self, state_str, gp_contacts, takk_data):
		self.grasp_states.append((state_str, gp_contacts, takk_data))

class GraspData:
	def __init__(self, grasp_num):
		self.grasp_num = grasp_num
		self.grasp_states = GraspStates()
		self.final_pose = PoseStamped()
		self.pose_to_world_transform = None
		self.trimesh = None
		self.stl_mesh_file = ""

	# Takes the directory in which to store this grasp
	#	needs a slash at the end.
	def write_file(dir_path):
		filename = dir_path + "grasp_" + str(self.grasp_num)
		out_file = open(filename, "w")

		grasp_data = pickle.dumps(self, 0)
		out_file.write(grasp_data)


current_grasp = GraspData(0)
cur_pose_list = None
cur_pose_selection = None
cur_gp_contacts = None
cur_takk_data = None
cur_openrave_params = None

def openrave_grasps_cb(pose_list):
	global cur_pose_list
	cur_pose_list = copy(pose_list)
	print "openrave grasps: ", cur_pose_list

def selected_grasp_cb(msg):
	global cur_pose_selection
	cur_pose_selection = copy(msg.grasp_id)
	print "selected grasp id", cur_pose_selection

def gp_data_cb(msg):
	global cur_gp_contacts
	cur_gp_contacts = copy(msg.pressure)
	print "cur_gp_contacts: ", cur_gp_contacts

def gp_called_cb(msg):
	global current_grasp
	global curr_takk_data
	rospy.loginfo("GP has been run. Recording gp contact data as grasp state.")

	current_grasp.grasp_states.add_state("closed", msg, cur_takk_data)

def takk_data_cb(msg):
	global cur_takk_data
	cur_takk_data = copy(msg.pressure)
	print "Got takktile data: ", cur_takk_data

def openrave_params_cb(msg):
	global cur_openrave_params
	cur_openrave_params = copy(msg)
	print "openrave params: centroid - ", cur_openrave_params.mesh_centroid

def init_logger():
	palm_usage = False
	num_fingers = 0
	while(True):
		user_input = raw_input("Are you using the palm? (y/n):")
		if user_input.lower() != "y" and user_input.lower() != "n":
			print "Unrecognized'", user_input.lower(), "' input. Use 'y' or 'n'"
			continue
		if user_input.lower() == "y":
			palm_usage = True
			break

		if user_input.lower() == "n":
			palm_usage = False
			break

	while(True):
		user_input = raw_input("How many fingers are attached? ")
		try:
			num_fingers = int(user_input)
			break
		except:
			print "Input was not a number"
			continue
	
	return palm_usage, num_fingers

	
def input_loop(using_palm, num_fingers):
	global current_grasp
	now = int(rospy.Time.now().to_sec())
	log_dir_path = hullify_base_path + "/logs/" + str(now)

	rospy.loginfo("log dir: %s" % log_dir_path)
	output_dir = os.mkdir(log_dir_path)
	grasp_num = 0
	while True:
		user_input = raw_input("Choose: n - next grasp, p - pregrasp state, m - manipulation state, f - finish testing: ")
		if user_input.lower() == "n":
			grasp_num += 1
			current_grasp.write_file(log_dir_path + "/")
			current_grasp = GraspData(grasp_num)
			rospy.logwarn("\tVerify that grasp %d is in the log directory." % grasp_num)

		elif user_input.lower() == 'p':
			global cur_takk_data
			global cur_gp_contacts
			current_grasp.grasp_states.add_state("pregrasp", cur_gp_contacts, cur_takk_data)
			rospy.loginfo("\tAdding pregrasp grasping state to record")

		elif user_input.lower() == "m":
			global cur_takk_data
			global cur_gp_contacts
			current_grasp.grasp_states.add_state("manipulation", cur_gp_contacts, cur_takk_data)
			rospy.loginfo("\tAdding manipluation grasping state to record")

		elif user_input.lower() == "f":
			rospy.loginfo("Finishing testing session.")

		else:
			rospy.loginfo("Unsupported operation: %s" % user_input)

def init_subscribers():
	grasp_selection_sub = rospy.Subscriber("/grasp_control/l_hand/grasp_selection", GraspSelection, selected_grasp_cb)

	openrave_poses_sub = rospy.Subscriber("/convex_hull/openrave_grasps", PoseArray, openrave_grasps_cb)

	openrave_params_sub = rospy.Subscriber("/convex_hull/openrave_params", Mesh_and_bounds, openrave_params_cb)

	takk_data_sub = rospy.Subscriber("/takktile/calibrated", Touch, takk_data_cb)

	gp_contacts_sub = rospy.Subscriber("/gp_contacts", Touch, gp_data_cb)

	#gp_called_sub = rospy.Subscriber("/grasp_gp/logging", , gp_called_cb)
	rospy.logwarn("gp_called sub not working yet, need to decide on a message type")

	return [grasp_selection_sub, openrave_poses_sub, openrave_params_sub, takk_data_sub, gp_contacts_sub]

if __name__ == "__main__":
	rospy.init_node("sprint4_logger")

	using_palm, num_fingers = init_logger()
	if using_palm:
		print "Using palm and ", num_fingers, " fingers."
	else:
		print "Using ", num_fingers, " fingers and no palm"
	
	subs = init_subscribers()

	input_loop(using_palm, num_fingers)
