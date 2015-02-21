#!/usr/bin/env python
import rospy
from openravepy import *
from openravepy.examples import tutorial_grasptransform
import rospkg
import os
import select
import numpy, time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose

import numpy
from numpy import pi, eye, dot, cross, linalg, sqrt, ceil, size
from numpy import hstack, vstack, mat, array, arange, fabs, zeros
import math
import random
import copy
import sys
import argparse

import grasping
import plane_filters
import atlas_and_ik
import openraveIO
import view_filter
import process_pool
import subprocess_comm

gt = None
world_axes = None
cur_hand = "l_robotiq"
arm_type = "L"
grasp_target_name = "grasp_target"
num_addtl_processes = 2

#Environment var name->[file_name, name_in_system]
FILE_PATH = 0
ENV_NAME = 1
loaded_hands = {"l_robotiq":['robots/robotiq.dae', '']}

class grasp_params:
	def __init__(self):
		return 1

	def __init__(self, convex_hull, openrave_params):
		self.convex_hull = convex_hull
		self.openrave_params = openrave_params

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
	target = get_grasp_target(env)

	env.SetViewer('qtcoin')
	gt = tutorial_grasptransform.GraspTransform(env,target)
	#view_robot_ref_frames(robot)
	
	show_atlas = rospy.get_param("/openrave/show_atlas", True)
	if not show_atlas:
		disable_atlas_visiblity(robot)

	return env, robot, target

def build_environment_subprocess():
	env = Environment()
	atlas_and_ik.load_atlas(env)
	robot = get_robot(env)

	env.Load('scenes/grasp_target.env.xml')
	target = get_grasp_target(env)

	return env, robot, target

def get_robot(env):
	using_atlas = rospy.get_param("convex_hull/using_atlas", True)
	robot = None
	if using_atlas:
		robot = env.GetRobot("atlas")
	else:
		robot = env.GetRobot("adept")

	if robot == None:
		print "Could not select specified robot. Is it in the environment?"

	return robot

def get_grasp_target(env):
	target = env.GetKinBody(grasp_target_name)
	if target == None:
		print "Could not obtain grasping target. Is it in the envionment?"
		sys.exit(1)

	return target

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

def get_final_pose_frame():
	try:
		final_pose_frame = rospy.get_param("convex_hull/output_pose_frame")
	except KeyError:
		rospy.logfatal("Parameter convex_hull/output_pose_frame could not be found. Are you using the standard launch files?")
		sys.exit(1)

	print "Output frame for poses: ", final_pose_frame
	return final_pose_frame

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

		#self.num_addtl_processes = num_addtl_processes

	def __del__(self):
		# Close the sockets to kill subprocesses
		self.close_sockets_and_kill_subprocesses()

	def __exit__(self, type, value, traceback):
		self.close_sockets_and_kill_subprocesses()

	def close_sockets_and_kill_subprocesses(self):
		rospy.logwarn("Killing subprocesses on cleanup.")
		for idx, process in enumerate(self.processes):
			process[1].shutdown()
			os.wait()

	def init_subprocesses(self):
		global num_addtl_processes
		print "Initalizing process pool. ", num_addtl_processes, " additional processes"
		#max_conn_count = 128
		#self.process_controller_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		#self.process_controller_socket.bind(('0.0.0.0', 0))
		#self.process_controller_socket.listen(max_conn_count)
		#self.process_socket_addr = self.process_controller_socket.getsockname()
		#print "OpenRAVE process controller socket initialized. Address: ", self.process_socket_addr
		max_conn_count = 128		
		self.process_controller_socket, self.process_socket_addr = subprocess_comm.start_master_socket(max_conn_count)		

		if num_addtl_processes > max_conn_count:
			rospy.logwarn("Requested process count is greater than the max connection count allowed for listen(). Reducing subprocess count to " + str(max_conn_count))
			num_addtl_processes = max_conn_count

		self.processes = []
		self.grasp_task_msgs = []

		# Fire up the subprocesses using fork() and system()
		for i in range(num_addtl_processes):	
			self.grasp_task_msgs.append(process_pool.grasp_params(None, None))
			
			fork_result = os.fork()
			if fork_result == -1:
				rospy.logerr("Fork() unsuccessful when creating OpenRAVE subprocesses.")
				num_addtl_processes -= 1

			elif fork_result != 0:
				print "Started subprocess PID: ", fork_result, ". Waiting for socket connection."

			else:
				rospy.logerr("NOTE: the communication sockets between processes uses the local loopback address, this will not work on a larger network.")
				subprocess_cmd_str = "rosrun rave_to_moveit SimEnvLoading.py --ip=" + str("127.0.0.1") + " --port=" + str(self.process_socket_addr[1])
				os.system(subprocess_cmd_str)
				sys.exit(1)
		
		# Collect connections
		for i in range(num_addtl_processes):
			#subprocess_sock_and_addr = self.process_controller_socket.accept()
			#child_pid = int(subprocess_sock_and_addr[0].recv(100))
			#self.processes.append([child_pid, subprocess_sock_and_addr[0]])
			cur_conn = subprocess_comm.subprocess_comm_master(self.process_controller_socket)
			cur_conn.set_state(subprocess_comm.GOOD)
			child_pid = int(cur_conn.read_max_payload())
			self.processes.append([child_pid, cur_conn])		
			print "Connection achieved: ", cur_conn.get_child_addr()

		print "Master's connection table: ", self.processes

	# Abstracting away process count incase layout changes again.
	def get_num_addtl_processes(self):
		return len(self.processes)

	def set_io(self, io_obj):
		self.raveio = io_obj

	def process_loop(self, master_addr):
		#global MAX_SOCKET_PAYLOAD
		self.raveio = None
		self.master_conn = subprocess_comm.subprocess_comm_slave(master_addr)

		while True:
			#Get the grasping params with blocking IO
			print "\tSubprocess Listening..."
			new_grasping_task = self.master_conn.listen_for_grasp_request()
			if new_grasping_task == None:
				continue

			self.replace_target(new_grasping_task.convex_hull)

			# Evaluate the grasps and report
			self.gmodel.generate(**new_grasping_task.openrave_params)
			self.master_conn.send_results(self.gmodel.grasps)
			print "\tFinished evaluating grasps. Good grasp count: ", len(self.gmodel.grasps)

	def update_process_targets(self, convex_hull):		
		# Add to all process targets
		for i in range(len(self.grasp_task_msgs)):		
			self.grasp_task_msgs[i].convex_hull = convex_hull

		# Update myself
		self.replace_target(convex_hull)		

	def replace_target(self, convex_hull):
		new_mesh = TriMesh()
		new_mesh.vertices = []
		for vertex in convex_hull.vertices:
			new_mesh.vertices.append([vertex.x, vertex.y, vertex.z])
	
		new_mesh.indices = []
		for triangle_mesh in convex_hull.triangles:
			new_mesh.indices.append(list(triangle_mesh.vertex_indices))

		grasp_target = get_grasp_target(self.env)
		self.env.RemoveKinBody(grasp_target)
	
		grasp_target.InitFromTrimesh(new_mesh, True)
		self.env.AddKinBody(grasp_target)

	def find_grasps(self, mesh_and_bounds_msg): 
		global ikmodel
		global gt

		self.totalgrasps = []

		params = plane_filters.generate_grasp_params(self.gmodel, mesh_and_bounds_msg)
		params['approachrays'] = view_filter.directional_filter(params['approachrays'], mesh_and_bounds_msg.mesh_centroid, raveio.pelvis_listener, 140)
		params['checkgraspfn'] = check_grasp_fn;

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

		# Clean up previous distribution
		self.clear_subprocess_connections()

		partitioned_rays = partition_rays_for_processes(params['approachrays'], self.get_num_addtl_processes() + 1)
		
		if sum([len(x) for x in partitioned_rays]) < 1:
			print "Insufficient approach rays generated. How do the bounding/filtering planes look? Returning null grasp list."
			return []


		# Dispatch the subprocesses
		params['remaininggrasps'] = -1	# Run through all grasps, no limit
		outstanding_results = []
		outstanding_processes = []
		for idx, process in enumerate(self.processes):
			self.grasp_task_msgs[idx].openrave_params = params
			self.grasp_task_msgs[idx].openrave_params['approachrays'] = partitioned_rays[idx]
			if (self.processes[idx][1].send_grasping_request(self.grasp_task_msgs[idx])):
				print "Process ", self.processes[idx][0], " was sent grasping parameters."
				outstanding_results.append(process[1].get_sock())
				outstanding_processes.append(idx)
			else:
				rospy.logerr("Could not send parameters to subprocess. Ignoring...")
				self.processes[idx][1].set_state(subprocess_comm.UNRESPONSIVE)


		# Get results on the local process
		#raw_input("Is the other process running?")
		params['approachrays'] = partitioned_rays[-1]
		self.gmodel.generate(**params)
		grasps.extend(self.gmodel.grasps)

		# Collect results
		grasps.extend(self.collect_subprocess_results(outstanding_results, outstanding_processes))

		return grasps
	
	def clear_subprocess_connections(self):
		num_removed_processes = 0
		for idx in range(len(self.processes)-1, -1, -1):
			process = self.processes[idx]
			if process[1].get_state() == subprocess_comm.UNRESPONSIVE:
				socket_survived = process[1].get_results()
				if socket_survived == None:
					process[1].shutdown()
					del(self.processes[idx])
					num_removed_processes += 1

		rospy.loginfo("Removed %d processes from master's process table.", num_removed_processes)


	def collect_subprocess_results(self, outstanding_results, outstanding_processes):
		subprocess_grasps = []
		process_collected_cnt = 0
		end_time = rospy.Time.now() + rospy.Duration(5)
		while len(outstanding_results) > 0:
			cur_time = rospy.Time.now()
			remaining_time = end_time.to_sec() - cur_time.to_sec()
			if remaining_time < 0:
				break

			rlist, wlist, xlist = select.select(outstanding_results, (), (), remaining_time)
			if len(rlist) == 0:
				# Timeout triggered this...
				break
			else:
				for conn in rlist:
					#print "conn: ", conn, " outstanding_results: ", outstanding_results
					idx = outstanding_results.index(conn)
					#print "idx: ", idx, " outstanding_processes: ", outstanding_processes
					proc_idx = outstanding_processes[idx]
					results = self.processes[proc_idx][1].get_results()
					if results is not None:
						process_collected_cnt += 1
						subprocess_grasps.extend(results)
					else:
						rospy.logerr("Ignoring grasping results.")
						ret = self.processes[proc_idx][1].dump_buffer()
						if ret == None:
							self.processes[proc_idx][1].set_state(subprocess_comm.UNRESPONSIVE)

					try:
						outstanding_results.remove(conn)
						outstanding_processes.remove(proc_idx)
					except ValueError as e:
						rospy.logerr("Attempting to remove the connection given by proc_idx %d and idx %d. Error, no such value.", proc_idx, idx)
		
		for proc_idx in outstanding_processes:
			self.processes[proc_idx][1].set_state(subprocess_comm.UNRESPONSIVE)

		rospy.loginfo("Read results from %d of %d processes.", process_collected_cnt, self.get_num_addtl_processes())
		rospy.loginfo("%d addtl grasps found by subprocesses.", len(subprocess_grasps))
		return subprocess_grasps

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
		partition.append(approach_rays[i * num_rays_each: (i+1) * num_rays_each])

	for i in range(leftover_rays):
		numpy.append(partition[i], approach_rays[-1 - i])

	print "Partition: ", partition
	return partition
	
def check_grasp_fn(contacts,finalconfig,grasp,analysis):
	if analysis['volume'] > 1e-6:
		return True
	else:
		return False


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
	


def parse_args():
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, help="If the instantiated process is a subprocess, this argument specifies the ip address of the master process.", default="0.0.0.0")
	parser.add_argument("--port", type=int, help="This option comes with the --ip option. It specifies the port of the master process.", default=0)

	args = parser.parse_args()
	ret_tuple = None
	if args.ip != "0.0.0.0" and args.port != 0:
		ret_tuple = (True, args.ip, args.port, "")
	elif args.ip != "0.0.0.0":
		print "Non-trivial ip specified with port 0, is this is a master process or a slave process?? Port:", args.port, " ip: ", args.ip, " Terminiating..."
		sys.exit(1)
	elif args.port != 0:
		print "Non-trivial port specified with wildcard ip. Is this a master process or a slave process?? Terminating..."
		sys.exit(1)
	else:
		ret_tuple = (False, "0.0.0.0", 0)
		
	return ret_tuple


if __name__ == '__main__':
	args = parse_args()
	if args[0] == True:
		# This is a slave process
		#sys.stdout = open("/home/eva/openrave_subprocess_log", "w")
		rospy.init_node('SimEnvLoading', anonymous=True)
		env, robot, target = build_environment_subprocess()
		grasper_clone = VigirGrasper(env, robot, target)
		grasper_clone.process_loop((args[1], args[2]))
	else:
		# This is the master process
		rospy.init_node('SimEnvLoading', anonymous=False)

		set_openrave_logging()
		set_openrave_environment_vars()
		env, robot, target = build_environment()
		grasper = VigirGrasper(env, robot, target)
		grasper.init_subprocesses()

		final_pose_frame = get_final_pose_frame()
		mesh_ref_frame = rospy.get_param("convex_hull/mesh_ref_frame")
		raveio = openraveIO.openraveIO(grasper, final_pose_frame, mesh_ref_frame)
	
		grasper.set_io(raveio)

		hand_subscriber = listen_for_LR_hand()

		print "Awaiting hulls..."
		rospy.spin()
