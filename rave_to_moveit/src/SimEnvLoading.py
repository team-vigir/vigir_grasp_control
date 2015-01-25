#!/usr/bin/env python
import rospy
from openravepy import *
from openravepy.examples import tutorial_grasptransform
import rospkg
import os
import numpy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose

import numpy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math
import random
import copy

from multiprocessing import Process, Queue

import grasping
import plane_filters
import atlas_and_ik
import openraveIO
import view_filter
#import check_initial_collisions as collision_test
#import plugin_mods_verification

gt = None
world_axes = None
cur_hand = "l_robotiq"
arm_type = "L"
grasp_target_name = "grasp_target"
num_addtl_processes = 3

#Environment var name->[file_name, name_in_system]
FILE_PATH = 0
ENV_NAME = 1
loaded_hands = {"l_robotiq":['robots/robotiq.dae', '']}

def set_openrave_logging():
	misc.InitOpenRAVELogging()	#Sync python logging
	RaveSetDebugLevel(DebugLevel.Verbose)
	RaveLogInfo("OpenRAVE Online")

def set_openrave_environment_vars():
	rospack = rospkg.RosPack()
	rave_to_moveit_path = rospack.get_path('rave_to_moveit')
	if os.environ.get("OPENRAVE_DATA", "") != "":
		os.environ["OPENRAVE_DATA"] = rave_to_moveit_path + ":" + os.environ["OPENRAVE_DATA"]
	else:
		os.environ["OPENRAVE_DATA"] = rave_to_moveit_path

	#print "set env vars"

	if os.environ.get("OPENRAVE_PLUGINS", "") != "":
		os.environ["OPENRAVE_PLUGINS"] = rave_to_moveit_path + "/plugins" + ":" + os.environ["OPENRAVE_PLUGINS"]
	else:
		os.environ["OPENRAVE_PLUGINS"] = rave_to_moveit_path + "/plugins"
	#print "Plugin path: ", os.environ["OPENRAVE_PLUGINS"]

#def load_modified_grasper_plugin(env):
	#plugin  = RaveLoadPlugin('grasper_mod')
#	plugin = RaveCreateModule(env,'Grasper')
#	if not plugin:
#		print "Could not load modified plugin for Grasper. Will default to standard."

def build_environment():
	global gt	
	env = Environment()
	atlas_and_ik.load_atlas(env)
	
	robot = get_robot(env)
	
	env.Load('scenes/grasp_target.env.xml')
	#target = env.GetKinBody(grasp_target_name)
	target = get_grasp_target(env)

	env.SetViewer('qtcoin')
	gt = tutorial_grasptransform.GraspTransform(env,target)
	#view_robot_ref_frames(robot)
	
	show_atlas = rospy.get_param("/openrave/show_atlas", True)
	if not show_atlas:
		disable_atlas_visiblity(robot)

	return env, robot, target

def get_robot(env):
	using_atlas = rospy.get_param("convex_hull/using_atlas", True)
	if using_atlas:
		return env.GetRobot("atlas")
	else:
		return env.GetRobot("adept")

def get_grasp_target(env):
	return env.GetKinBody(grasp_target_name)

def disable_atlas_visiblity(robot):
	print "Disabling Atlas visiblity."
	robot.SetVisible(False)
	
	draw_world_axes(robot)

	#Reenable the manipulator visibility
	manipulators = robot.GetManipulators()
	for manip in manipulators:
		manip_links = manip.GetChildLinks()
		for link in manip_links:
			link.SetVisible(True)

def view_robot_ref_frames(robot):
	global gt
	a2 = gt.drawTransform(robot.GetActiveManipulator().GetEndEffector().GetTransform(), length=0.2)
	
	draw_world_axes(robot)

	raw_input("Drew robot transform frame. Pausing before leaving scope...")
	
	#atlas_and_ik.test_transforms(gt)

def draw_world_axes(robot):
	global world_axes
	world_axes =  gt.drawTransform(robot.GetTransform(), length=0.3)

#def load_hands():
#	global env
#	global loaded_hands
#
#	for hand_name in loaded_hands:
#		robot = env.ReadRobotXMLFile(loaded_hands[hand_name][FILE_PATH])
#		if robot is None:
#			print "Cannot load robot: ", loaded_hands[hand_name][FILE_PATH]
#		else:
#			env.AddRobot(robot)
#			loaded_hands[hand_name][ENV_NAME] = robot.GetName()
#			print "Loaded ", loaded_hands[hand_name][ENV_NAME]

def query_final_pose_frame():
#	while True:
#		suggestion = raw_input("Please input the reference frame of the final pose: \n\t0 - new \n\t1 - /world: ")
#		if suggestion == "0":
#			final_pose_frame = raw_input("what's your new frame? (please give it a slash prefix")
#		elif suggestion == "1":
#			final_pose_frame = "/world"
#
#		else:
#			continue
#
#		break
	final_pose_frame = rospy.get_param("convex_hull/output_pose_frame")
		#print("Could not get param: convex_hull/output_pose_frame, exiting")
		#exit(1)

	print "Output frame for poses: ", final_pose_frame
	return final_pose_frame

class graspProcess:
	def __init__(self):
		print "Default constructor called for graspProcess"
	def __init__(self, funct, env_copy, param_pipe, result_queue):
		args = (env_copy, param_pipe, result_queue, )		
		self.process = Process(target=funct, args=args)
		self.param_pipe = param_pipe
		self.result_queue = result_queue

		process.start()
	
	def evaluate_grasps(params):
		self.param_pipe.put(params)

class VigirGrasper:
	def __init__(self, env, robot, target):
		print "Making grasper"
		global num_addtl_processes		
		self.env = env
		self.robot = robot
		self.target = target
		self.gmodel = grasping.GraspingModel(robot,target)
		self.totalgrasps = []
		self.raveio = None
		self.grasp_returnnum = rospy.get_param("/convex_hull/openrave_grasp_count_goal", 20);

		self.num_addtl_processes = num_addtl_processes
		init_subprocesses()

	def init_subprocesses(self):
		print "Initalizing process pool"
		self.grasp_task_msgs = []
		self.processes = []
		for i in range(self.num_addtl_processes):		
			self.grasp_task_msgs.append(process_pool.grasp_params())
			self.processes.append(graspProcess(process_pool.process_loop, env.CloneSelf(openravepy_int.CloningOptions.Bodies | openravepy_int.CloningOptions.Modules), Queue(), Queue()) 

	def set_io(self, io_obj):
		self.raveio = io_obj

	def update_process_targets(self, convex_hull):		
		# Add to all process targets
		for i in range(len(self.grasp_task_msgs)):		
			self.grasp_task_msgs[i].convex_hull = convex_hull

		# Update myself
		process_pool.replace_target(self.env, convex_hull)		

	#def replace_target(self, convex_hull):
	#	new_mesh = TriMesh()
	#	new_mesh.vertices = []
	#	for vertex in convex_hull.vertices:
	#		new_mesh.vertices.append([vertex.x, vertex.y, vertex.z])
		#print "convex_hull indices: ", convex_hull.triangles
	#	new_mesh.indices = []
	#	for triangle_mesh in convex_hull.triangles:
	#		new_mesh.indices.append(list(triangle_mesh.vertex_indices))

		#print new_mesh.indices
		#print new_mesh.vertices
		#print dir(new_mesh)
	#	grasp_target = self.env.GetKinBody('grasp_target')
	#	self.env.RemoveKinBody(grasp_target)
	#	grasp_target.InitFromTrimesh(new_mesh, True)
	#	self.env.AddKinBody(grasp_target)

		#lol = raw_input("Pausing...")

	def find_grasps(self, mesh_and_bounds_msg): 
		global ikmodel
		global gt

		self.totalgrasps = []

		params = plane_filters.generate_grasp_params(self.gmodel, mesh_and_bounds_msg)
		params['approachrays'] = view_filter.directional_filter(params['approachrays'], mesh_and_bounds_msg.mesh_centroid, raveio.pelvis_listener, 140)
		params['checkgraspfn'] = self.check_grasp_fn;

		raveio.enable_moveit_octomap_updating(False)
		
		self.totalgrasps = self.get_grasps(mesh_and_bounds_msg, params, gt, returnnum=self.grasp_returnnum)
		
		raveio.enable_moveit_octomap_updating(True)
		
		if len(self.totalgrasps) == 0:
			print "No suitable grasps found. Please select another pointcloud."
			return

		graspnum = len(self.totalgrasps)
		print graspnum, " Grasps available."
		#self.show_grasps(self.totalgrasps)

		pose_array = []
		offset = rospy.get_param("/convex_hull/pregrasp_offset")
		with robot:
			x = 0
			while x < graspnum:
				grasp = self.totalgrasps[x]
				T = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
				p = raveio.TransformToPoseStamped(T)
				#print "p before: ", p
				pp = self.get_pregrasp_pose(copy.deepcopy(p), grasp[self.gmodel.graspindices.get('igraspdir')], offset)
				#print "p after: ", p
				pose_array.append(p)
				pose_array.append(pp)

				x += 1
		
			#print "pose_array: ", pose_array

		self.raveio.publish_poses(pose_array)
		if rospy.get_param("convex_hull/openrave_show_grasps"):
			self.show_selected_grasps(self.totalgrasps)

		if not rospy.get_param("convex_hull/openrave_show_ik"):
			self.show_ik_on_request()

	def get_grasps(self, mesh_and_bounds_msg, params, gt, returnnum=5):
		grasps = []
		#partitioned_rays = partition_rays(mesh_and_bounds_msg, params['approachrays'])	#90 plane filtering
		#partitioned_rays = [params['approachrays']]
		partitiioned_rays = partition_rays_for_processes(params['approachrays'], self.num_addtl_processes + 1)
		
		if sum([len(x) for x in partitioned_rays]) < 1:
			print "Insufficient approach rays generated. How do the bounding/filtering planes look? Returning null pose."
			return []


		#for rays in partitioned_rays:
		#	params['approachrays'] = rays
		#	params['remaininggrasps'] = returnnum - len(grasps)
			#atlas_and_ik.visualize_approaches(gt, params)
			
			#plugin_mods_verification.show_first_for_check_distance(params, gt)
			
		#	self.gmodel.generate(**params)
		#	grasps.extend(self.gmodel.grasps)

		#	if len(grasps) >= returnnum:
		#		return grasps
		
		# Dispatch the subprocesses
		for idx, process in enumerate(self.processes):
			self.grasp_task_msgs[idx].openrave_params = params
			self.grasp_task_msgs[idx].openrave_params['approach_rays'] = partitioned_rays[idx]
			self.processes[idx].evaluate_grasps(self.grasp_task_msgs[idx])

		# Get results on the local process
		params['approach_rays'] = partitioned_rays[-1]
		self.gmodel.generate(**params)
		grasps.extend(self.gmodel.grasps)

		# Collect results
		outstanding_results = range(self.num_addtl_processes)
		while len(outstanding_results) != 0:
			for i in outstanding_results:
				if not self.processes[i].result_queue.empty():
					print "Got results from process ", i
					grasps.extend(self.processes[i].result_queue.get()
					del outstanding_results[i]
			rospy.sleep(0.1)

		return grasps
	
	def check_grasp_fn(self,contacts,finalconfig,grasp,analysis):
		if analysis['volume'] > 1e-6:
			return True
		else:
			return False

	# The approach_vector must be the direction the hand moves toward the object!
	def get_pregrasp_pose(self, final_pose_stamped, approach_vec, offset):
		pre_grasp_pose_stamped = PoseStamped(final_pose_stamped.header, final_pose_stamped.pose)
		
		if offset > 0:
			offset = -offset
		
		# Normalize approach
		sq_sum = (approach_vec[0]**2 + approach_vec[1]**2 + approach_vec[2]**2)**(1/2)
		offset_vec = [(offset * x)/sq_sum for x in approach_vec]

		print "Offset vec: ", offset_vec, " approach_vec: ", approach_vec
		pre_grasp_pose_stamped.pose.position.x += offset_vec[0]
		pre_grasp_pose_stamped.pose.position.y += offset_vec[1]
		pre_grasp_pose_stamped.pose.position.z += offset_vec[2]

		return pre_grasp_pose_stamped

	def show_grasps(self, grasps):
		for grasp in grasps:
			self.gmodel.showgrasp(grasp)

	def show_selected_grasps(self, grasps):
		while True:
			print "There are ", len(grasps), " available (zero indexed) please select one or -1 to quit: "
			res = raw_input()
			if res == "":
				continue
			if res == "-1" or res == "q" or res == "Q":
				break

			res = int(res)
			if res < 0 or res >= len(grasps):
				print "Improper numeric value. Remember, it's zero indexed."
				continue
			a1 = self.show_grasp_transform(gt, grasps[res])
			self.gmodel.showgrasp(grasps[res])

	def show_grasp_transform(self, gt, grasp):
		Tgrasp = self.gmodel.getGlobalGraspTransform(grasp, collisionfree=True)
		return gt.drawTransform(Tgrasp, length=0.2)

	def show_ik_on_request(self):
		#while True:
			#res = raw_input("Input the index of the grasp you would like IK for (q to quit): ")
			#if res == "q" or res == "Q":
			#	break
			#transform = self.gmodel.getGlobalGraspTransform(self.totalgrasps[int(res)],collisionfree=True)
			#atlas_and_ik.visualize_ik_solution(self.env, transform)

		#atlas_and_ik.display_moveitik_results(self.raveio.ikresults, self.robot)
		self.raveio.ikresults = []
	

def partition_rays(mesh_and_bounds_msg, rays):
	partitioned_rays = []
	sweet_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.ninety_degree_bounding_planes[0].coef, mesh_and_bounds_msg.ninety_degree_bounding_planes[1].coef, False)

	partitioned_rays.append(rays.take(sweet_idxs, axis=0))

	wider_idxs = plane_filters.filter_bounding_planes(rays, mesh_and_bounds_msg.knowledge_bounding_planes[0].coef, mesh_and_bounds_msg.knowledge_bounding_planes[1].coef, mesh_and_bounds_msg.plane_sep_angle_gt_pi)
	
	partitioned_rays.append(rays.take([x for x in wider_idxs if x not in sweet_idxs], axis=0))

	#print partitioned_rays
	#print "sweet_shape: ", partitioned_rays[0].shape, " wider_shape: ", partitioned_rays[1].shape
	#raw_input("How does that partition look?")

	#print "plane1 ", mesh_and_bounds_msg.ninety_degree_bounding_planes[0].coef, " plane2: ", mesh_and_bounds_msg.ninety_degree_bounding_planes[1].coef
	#print "Sweet point: ", partitioned_rays[0][0]
	#raw_input("Is that point filtered properly?")

	return select_from_partition(partitioned_rays)

def select_from_partition(partitioned_rays):
	print "Picking out a certain number of approach rays!"
	num_sweet_idxs = 55
	num_ok_idxs = 8
	if len(partitioned_rays[0]) > num_sweet_idxs:
		numpy.random.shuffle(partitioned_rays[0])
		partitioned_rays[0] = partitioned_rays[0][:num_sweet_idxs]
	else:
		num_ok_idxs = 65 - len(partitioned_rays[0])
		

	if len(partitioned_rays[1]) > num_ok_idxs:
		numpy.random.shuffle(partitioned_rays[1])
		partitioned_rays[1] = partitioned_rays[1][:num_ok_idxs]


	return partitioned_rays

def partition_rays_for_processes(approach_rays, total_num_evaluators):
	partition = []
	num_rays = len(approach_rays)
	num_rays_each = num_rays / (total_num_evaluators)
	leftover_rays = num_rays % total_num_evaluators

	for i in range(total_num_evaluators):
		partition.append(approach_rays[i * num_rays_each: (i+1) * num_rays_each]

	for i in range(leftover_rays):
		numpy.append(partition[i], approach_rays[-1 - i])

	print "Partition: ", partition
	raw_input("How does that partition look?")
	return partition

def listen_for_LR_hand():
	return rospy.Subscriber("grasping_hand_selection", String, set_hand_callback)

def set_hand_callback(msg):
	global cur_hand
	global loaded_hands
	global arm_type

	if msg.data == "L" or msg.data == "l":
		hand_type = os.environ.get("FLOR_LEFT_HAND_TYPE", "")
		arm_type = "L"

	elif msg.data == "R" or msg.data == "r":
		hand_type = os.environ.get("FLOR_RIGHT_HAND_TYPE", "")
		arm_type = "R"
	else:
		print "Unsupported arm selection in OpenRAVE/set_hand_callback(): ", msg.data

	if hand_type in loaded_hands:
		print "New hand in use: ", hand_type
		cur_hand = hand_type
	else:
		print "Unsupported hand: ", hand_type, " reusing current hand: ", cur_hand
		print "Please add ", hand_type, " to the loaded_hands dictionary in SimEnvLoading.py"
	


if __name__ == '__main__':
	rospy.init_node('SimEnvLoading', anonymous=False)

	set_openrave_logging()
	set_openrave_environment_vars()
	env, robot, target = build_environment()
	grasper = VigirGrasper(env, robot, target)
	
	final_pose_frame = query_final_pose_frame()
	mesh_ref_frame = rospy.get_param("convex_hull/mesh_ref_frame")
	raveio = openraveIO.openraveIO(grasper, final_pose_frame, mesh_ref_frame)
	
	grasper.set_io(raveio)

	hand_subscriber = listen_for_LR_hand()

	print "Awaiting hulls..."
	rospy.spin()
