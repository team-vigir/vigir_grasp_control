from openravepy import *
import numpy
import grasping
import SimEnvLoading

class grasp_params:
	def __init__(self):
		return 1

	def __init__(self, convex_hull, openrave_params):
		self.convex_hull = convex_hull
		self.openrave_params = openrave_params

def process_loop(env, param_pipe, result_queue):
	print "Process online!"
	robot = SimEnvLoading.get_robot(env)
	target = SimEnvLoading.get_grasp_target(env)

	gmodel = grasping.GraspingModel(robot,target)

	while True:
		#Get the grasping params with blocking IO
		print "Listening..."
		new_grasping_task = param_pipe.get(block=True, timeout=None)
		print "About to replace target!!"
		replace_target(env, new_grasping_task.convex_hull)
		print "\tTarget hash in subprocess: ", target.GetKinematicsGeometryHash()

		# Evaluate the grasps and report
		print "Before generation in subprocess"
		gmodel.generate(**new_grasping_task.openrave_params)
		print "\tPlaced the results in the output queue."
		result_queue.put(gmodel.grasps)
		print "\tFinished evaluating grasps. Good grasp count: ", len(gmodel.grasps)

def replace_target(env, convex_hull):
	new_mesh = TriMesh()
	new_mesh.vertices = []
	for vertex in convex_hull.vertices:
		new_mesh.vertices.append([vertex.x, vertex.y, vertex.z])
	
	new_mesh.indices = []
	for triangle_mesh in convex_hull.triangles:
		new_mesh.indices.append(list(triangle_mesh.vertex_indices))

	grasp_target = SimEnvLoading.get_grasp_target(env)
	env.RemoveKinBody(grasp_target)
	
	grasp_target.InitFromTrimesh(new_mesh, True)
	env.AddKinBody(grasp_target)
