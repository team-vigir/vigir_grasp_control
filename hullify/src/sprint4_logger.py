#!/usr/bin/env python
import rospy
from osu_grasp_msgs.msg import Mesh_and_bounds
from shape_msgs.msg import Mesh
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker
from takktile_ros.msg import Touch
from vigir_grasp_msgs.msg import GraspSelection
from copy import deepcopy as copy

import tf
from tf import transformations

import pickle
import os
import rospkg
rospack = rospkg.RosPack()
hullify_base_path = rospack.get_path("hullify")


class GraspStates:
	def __init__(self):
		self.grasp_states = []
	
	def add_state(self, state_str, gp_contacts, takk_data):
		self.grasp_states.append((state_str, gp_contacts, takk_data))

class GraspData:
	def __init__(self, grasp_num):
		self.grasp_num = grasp_num
		self.grasp_states = GraspStates()
		self.final_pose = PoseStamped()
		self.pregrasp_pose_offset = 0
		self.pose_to_world_transform = None
		self.mesh_to_world_transform = None
		self.mesh_msg = Mesh()
		#self.stl_mesh_file = ""
		self.gp_results = [0,0]
		self.success = 0

	# Takes the directory in which to store this grasp
	#	needs a slash at the end.
	def write_file(self,dir_path):
		filename = dir_path + "grasp_" + str(self.grasp_num)
		out_file = open(filename, "w")

		grasp_data = pickle.dumps(self, 0)
		out_file.write(grasp_data)
	def print_state(self):
		print "\tGraspStates: ", self.grasp_states.grasp_states
		print "\tPregrasp Pose Offset: ", self.pregrasp_pose_offset
		print "\tFinal pose: ", self.final_pose
		print "\tTransforms: \n\t\tpose_to_world: ", self.pose_to_world_transform, "\n\t\tmesh_to_world", self.mesh_to_world_transform
		print "\tMesh num triangles: ", len(self.mesh_msg.triangles)
		print "\tSuccess: ", self.success
		print "\tGP_results: ", self.gp_results


current_grasp = GraspData(0)
cur_pose_list = None
cur_pose = None
cur_gp_contacts = None
cur_takk_data = None
cur_openrave_params = None


def openrave_grasps_cb(pose_list):
	global cur_pose_list
	cur_pose_list = copy(pose_list)
	print "openrave grasps: ", cur_pose_list
	print "num poses received: ", len(cur_pose_list.poses)

def grasp_selected_cb(msg):
	global cur_pose
	cur_pose = copy(msg)
	print "selected grasp \n", cur_pose

def grasp_executed_cb(msg):
	# Record all parameter information
	global current_grasp
	global cur_pose
	current_grasp.final_pose = cur_pose
	mesh_T, final_grasp_T = get_grasp_transforms()
	print "mesh and final_grasp transforms: ", mesh_T, final_grasp_T
	current_grasp.pose_to_world_transform = final_grasp_T
	current_grasp.mesh_to_world_transform = mesh_T
	current_grasp.pregrasp_pose_offset = rospy.get_param("/convex_hull/pregrasp_offset")

def get_grasp_transforms():
	global current_grasp
	global cur_pose_list
	global cur_pose
	mesh_ref_frame = rospy.get_param("convex_hull/mesh_ref_frame")
	pose_ref_frame = cur_pose.header.frame_id

	print "mesh_ref_frame: ", mesh_ref_frame
	print "pose_ref_frame: ", pose_ref_frame

	mesh_trans, mesh_rot = current_grasp.mesh_to_world_transform = get_world_transform(mesh_ref_frame)
	pose_trans, pose_rot = current_grasp.pose_to_world_transform = get_world_transform(pose_ref_frame)

	return ((mesh_trans, mesh_rot), (pose_trans, pose_rot))

def get_world_transform(src_frame):
	global trans_listener
	freq = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			(trans, rot) = trans_listener.lookupTransform(src_frame, "/world", rospy.Time(0))
			break
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Lookup exception handled..."
			freq.sleep()
		except:
			print "Unexpected exception. Args: ", src_frame, " /world ", rospy.Time(0)
	return trans, rot	

def gp_data_cb(msg):
	global cur_gp_contacts
	cur_gp_contacts = copy(msg.pressure)
	#print "cur_gp_contacts: ", cur_gp_contacts

def gp_called_cb(msg):
	global current_grasp
	global curr_takk_data
	rospy.loginfo("GP has been run. Recording gp contact data as grasp state.")

	gp_data = msg.pressure[0:8]

	current_grasp.grasp_states.add_state("closed", gp_data, cur_takk_data)
	current_grasp.gp_results = msg.pressure[8:]

	success = raw_input("Was grasp successful (1/0): ")
	current_grasp.success = success

def takk_data_cb(msg):
	global cur_takk_data
	cur_takk_data = copy(msg.pressure)
	#print "Got takktile data: ", cur_takk_data

def openrave_params_cb(msg):
	global cur_openrave_params
	global current_grasp
	cur_openrave_params = copy(msg)
	print "openrave params: centroid - ", cur_openrave_params.mesh_centroid
	current_grasp.mesh_msg = msg.convex_hull

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
	global cur_takk_data
	global cur_gp_contacts
	now = int(rospy.Time.now().to_sec())
	log_dir_path = hullify_base_path + "/logs/grasp" + str(now)

	rospy.loginfo("log dir: %s" % log_dir_path)
	output_dir = os.mkdir(log_dir_path)
	grasp_num = 0
	while True:
		user_input = raw_input("Choose: pr - print current state, n - next grasp, p - pregrasp state, m - manipulation state, f - finish testing: ")
		if user_input.lower() == "n":
			grasp_num += 1
			current_grasp.write_file(log_dir_path + "/")
			current_grasp = GraspData(grasp_num)
			rospy.logwarn("\tVerify that grasp %d is in the log directory." % (grasp_num - 1))

		elif user_input.lower() == 'pr':
			current_grasp.print_state()

		elif user_input.lower() == 'p':
			current_grasp.grasp_states.add_state("pregrasp", cur_gp_contacts, cur_takk_data)
			rospy.loginfo("\tAdding pregrasp grasping state to record")

		elif user_input.lower() == "m":
			current_grasp.grasp_states.add_state("manipulation", cur_gp_contacts, cur_takk_data)
			rospy.loginfo("\tAdding manipluation grasping state to record")

		elif user_input.lower() == "f":
			rospy.loginfo("Finishing testing session.")
			current_grasp.write_file(log_dir_path + "/")
			break

		else:
			rospy.loginfo("Unsupported operation: %s" % user_input)

def init_subscribers():
	grasp_selected_sub = rospy.Subscriber("/convex_hull/grasp_select_logger", PoseStamped, grasp_selected_cb)
	
	grasp_executed_sub = rospy.Subscriber("/grasp_control/l_hand/grasp_selection", GraspSelection, grasp_executed_cb)

	openrave_poses_sub = rospy.Subscriber("/convex_hull/openrave_grasps", PoseArray, openrave_grasps_cb)

	openrave_params_sub = rospy.Subscriber("/convex_hull/openrave_params", Mesh_and_bounds, openrave_params_cb)

	takk_data_sub = rospy.Subscriber("/takktile/calibrated", Touch, takk_data_cb)

	gp_contacts_sub = rospy.Subscriber("/gp_contacts", Touch, gp_data_cb)

	
	gp_called_sub = rospy.Subscriber("/gp_logger", Touch, gp_called_cb)
	#rospy.logwarn("gp_called sub not working yet, need to decide on a message type")

	return [grasp_selected_sub, grasp_executed_sub, openrave_poses_sub, openrave_params_sub, takk_data_sub, gp_contacts_sub, gp_called_sub]

if __name__ == "__main__":
	rospy.init_node("sprint4_logger")
	trans_listener = tf.TransformListener()

	using_palm, num_fingers = init_logger()
	if using_palm:
		print "Using palm and ", num_fingers, " fingers."
	else:
		print "Using ", num_fingers, " fingers and no palm"
	
	subs = init_subscribers()

	input_loop(using_palm, num_fingers)
