import random
from openravepy.interfaces import TaskManipulation
from numpy import array
from numpy import linspace
from numpy import arange
import math

def generate_grasp_params(gmodel, mesh_and_bounds_msg):
	params = get_params(gmodel)

	#filtered_ray_idxs = filter_approach_rays(params['approachrays'], mesh_and_bounds_msg, num_return_rays=0)
	params['rolls'] = limit_wrist_rolling() #Forrest, please replace this!
	#params['rolls'] = limit_wrist_rolling_circle(30, params['approachrays'], gmodel.target, gmodel.env)
	params['preshapes'] = set_preshape(gmodel)
	params['manipulatordirections'] = set_manip_approach_direction(gmodel)
	#params['approachrays'] = gmodel.computeBoxApproachRays()
	#print dir(params['approachrays'])
	#print "__doc__: ", params['approachrays'].__doc__
	#print "__module__: ", params['approachrays'].__mod__#, params['approachrays'].__module__
	#print "base: ", params['approachrays'].bases
	#print params['approachrays']
	#print filtered_ray_idxs
	
	#print "standoffs: ", params['standoffs']
	#print "rolls: ", params['rolls']
	#params['approachrays'] = params['approachrays'].take(filtered_ray_idxs, axis=0)
	#print params['approachrays']
	#print params['graspingnoise']
	#params['graspingnoise'] = (0, 0)
	#params['normalanglerange'] = 0
	#params['spheredelta'] = 0
	params['standoffs'] = array([0])
	params['finestep'] = 0.01
	params['friction'] = 0.1
	#params['directiondelta'] = 0
	#gmodel.numthreads = 3

	#gmodel.generate(**params)
	return params

def get_params(gmodel):
	preshapes,standoffs,rolls,approachrays, graspingnoise,forceclosure,forceclosurethreshold,checkgraspfn,manipulatordirections,translationstepmult,finestep,friction,avoidlinks,plannername = gmodel.autogenerateparams()

	all_locals = locals()
	all_params = {}
	for key, value in all_locals.iteritems():
		if key != 'gmodel':
			all_params[key] = value

	return all_params

def filter_approach_rays(initial_approachrays, bounding_plane_msg, num_return_rays=1):
	plane1 = bounding_plane_msg.ninety_degree_bounding_planes[0].coef
	plane2 = bounding_plane_msg.ninety_degree_bounding_planes[1].coef
	planes_are_obtuse = bounding_plane_msg.plane_sep_angle_gt_pi

	#The ninety degree planes will never be obtuse
	cur_ray_idxs = filter_bounding_planes(initial_approachrays, plane1, plane2, False)

	cur_ray_idxs = random_ray_selection(cur_ray_idxs, num_return_rays)

	return cur_ray_idxs

def filter_bounding_planes(initial_approach_rays, bplane1, bplane2, planes_are_obtuse):
	out_ray_idxs = []
	for idx, ray in enumerate(initial_approach_rays):
		pt_to_bplane1_dist = get_plane_dist(ray[0:3], bplane1)
		pt_to_bplane2_dist = get_plane_dist(ray[0:3], bplane2)

		if pt_to_bplane1_dist >= 0 and pt_to_bplane2_dist >= 0:
			print ray, " inside both bounding planes."
			out_ray_idxs.append(idx)

		elif planes_are_obtuse and (pt_to_bplane1_dist >= 0 or pt_to_bplane2_dist >= 0):
			print ray, " inside one of the obtuse planes."
			out_ray_idxs.append(idx)
		else:
			print ray, " is not inside the bounded region. dist1: ", pt_to_bplane1_dist, " dist2: ", pt_to_bplane2_dist

	print "Initial ray count: ", len(initial_approach_rays), " final count: ", len(out_ray_idxs)
	return out_ray_idxs

def random_ray_selection(ray_idxs, num_return_rays):
	print "num_return_rays: ", num_return_rays
	raw_input("is it good?")
	if num_return_rays > len(ray_idxs):
		print "Insufficient approach ray count: ", len(ray_idxs), " rays available and asked to select ", num_return_rays
		return ray_idxs
	elif num_return_rays == 0:
		return ray_idxs

	return random.sample(ray_idxs, num_return_rays)

# The normal of the planes should point TOWARD THE ROBOT
#	A positive plane distance is included in the result rays
def filter_plane(rays, plane_coefficient_list):
	out_rays = []
	for ray in rays:
		#print ray, plane_coefficient_list
		pt_to_plane_dist = get_plane_dist(ray[0:3], plane_coefficient_list)
		print "For point ", ray[0:3], " the point to plane distance is ", pt_to_plane_dist
		if pt_to_plane_dist >= 0:
			print "\tPoint is valid."
			out_rays.append(ray)

		else:
			print "\tPoint is invalid."

	return out_rays

def get_plane_dist(pt, plane_coefficient_list):
	dot = 0
	norm_len = 0;
	for idx in range(3):
		#print "Is it a list?: ", pt[idx], " ", plane_coefficient_list[idx]
		dot += (pt[idx] * plane_coefficient_list[idx])
		norm_len += plane_coefficient_list[idx] ** 2

	norm_len = norm_len ** 0.5
	dist = (dot + plane_coefficient_list[3]) / norm_len
	return dist

#This is the number of wrist rolls to attempt around a given approach vector
#	??What is this relative to? Current position?
def limit_wrist_rolling():
	wrist_orientations = linspace(-math.pi / 2, math.pi / 2, num=5)
	print "wrist orientations: ", wrist_orientations
	return wrist_orientations
## Here is the function to replace...
## Recall that atlas_and_ik.py has functions to reorient the palm to the approach, so that we
## Input->	angle_check: angle interval at which to check for intersection with object
## Output->	wrist_orientations: array of wrist orientations to check (in radians)
def limit_wrist_rolling_circle(angle_check, approach_rays, target, env):
	if 180 % angle_check != 0:
		print "ERROR: Invalid angle input. Please choose an angle that is a factor of 180 degrees."
		return
	
	angle_check_radians = (angle_check/float(360)) * 2*math.pi
	angle_check_radians_fingers = arange(0, math.pi, angle_check_radians)
	#angle_check_radians_thumb = arange(math.pi, 2*math.pi, angle_check_radians)

	reach = 0.1524/2 #open hand reach in meters
	#angle_check_radians_pairs = zip(angle_check_radians_fingers, angle_check_radians_thumb)
	
	#Using one array of angles
	for approach_ray in approach_rays:
		for angle in angle_check_radians_fingers:
			##Calculate points 180 degrees from each other to check for collisions
			direction_vector = [math.cos(angle), 0, math.sin(angle)]
			magnitude = math.sqrt(sum([x**2 for x in direction_vector]))
			unit_vector = [x/magnitude for x in direction_vector]
			print "Unit Vector: ", unit_vector
			##First direction
			direction_vector_1 = [x*reach for x in unit_vector]
			check_ray_pos_1 = [x[0] + x[1] for x in zip(approach_ray[0:3], direction_vector_1)]
			check_ray_1 = check_ray_pos_1
			check_ray_1.extend(approach_ray[3:6])
			print "Original direction: ", approach_ray[3:6]
			print "First ray to check:\nPosition: ", check_ray_1[0:3], "\nDirection: ", check_ray_1[3:6]
			##Second direction
			direction_vector_2 = [x*-1*reach for x in unit_vector]
			check_ray_pos_2 = [x[0] + x[1] for x in zip(approach_ray[0:3], direction_vector_2)]
			check_ray_2 = check_ray_pos_2
			check_ray_2.extend(approach_ray[3:6])
			print "Second ray to check:\nPosition: ", check_ray_2[0:3], "\nDirection: ", check_ray_2[3:6]
			##Check for collisions
			if(env.CheckCollisionRays(check_ray_1, target) and env.CheckCollisionRays(check_ray_2, target)):
				wrist_orientations.append(angle)
				print "Angle to be added: ", angle
	print "Wrist Orientation Angles: ", wrist_orientations
	raw_input("How do those grasping params look?")
	return wrist_orientations

#Get an initial state for the fingers.
#	For the robotiq hand, this can be all zeros.
def set_preshape(gmodel):
	with gmodel.target:
		gmodel.target.Enable(False)
		taskmanip = TaskManipulation(gmodel.robot)
		final_joints, traj = taskmanip.ReleaseFingers(execute=False, outputfinal=True)
	#final_joints[-1] = 0.5
	print "Setting preshape of grasp to: ", final_joints
	return array([final_joints])

def set_manip_approach_direction(gmodel):
	manip_dirs = array([gmodel.manip.GetDirection()])
	print "manip_dirs: ", manip_dirs
	return manip_dirs
