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
		#Get the grasping params		
		new_grasping_task = param_pipe.get(block=True, timeout=None)
		replace_target(env, new_grasping_task.convex_hull)
		print "\tTarget hash in subprocess: ", target.GetKinematicsGeometryHash()

		# Evaluate the grasps
		gmodel.generate(**new_grasping_task.openrave_params)
		result_queue.put(gmodel.grasps)
		print "\tFinished evaluating grasps. Good grasp count: ", len(gmodel.grasps)

def replace_target(env, convex_hull):
	new_mesh = TriMesh()
	new_mesh.vertices = []
	for vertex in convex_hull.vertices:
		new_mesh.vertices.append([vertex.x, vertex.y, vertex.z])
	#print "convex_hull indices: ", convex_hull.triangles
	new_mesh.indices = []
	for triangle_mesh in convex_hull.triangles:
		new_mesh.indices.append(list(triangle_mesh.vertex_indices))

	#print new_mesh.indices
	#print new_mesh.vertices
	#print dir(new_mesh)
	grasp_target = self.env.GetKinBody('grasp_target')
	env.RemoveKinBody(grasp_target)
	
	grasp_target.InitFromTrimesh(new_mesh, True)
	env.AddKinBody(grasp_target)

	#lol = raw_input("Pausing...")
