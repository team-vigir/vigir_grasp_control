import vector_math
import rospy

#This function will remove approach vectors based not on the bounding planes
#	but on the angle between the approach ray and the camera_to_centroid vector
def directional_filter(approach_rays, centroid_pt, transform_listener, final_ray_count=0):
	acceptable_angle = 1.92 #rad
	centroid_vec = [centroid_pt.x, centroid_pt.y, centroid_pt.z]
	camera_to_centroid = get_camera_to_centroid(centroid_vec, transform_listener)
	filtered_rays = apply_directional_filter(camera_to_centroid, approach_rays, acceptable_angle)

	#trimmed_filtered_rays = random_selection(filtered_rays, final_ray_count)
	#return trimmed_filtered_rays
	return filtered_rays

def get_camera_to_centroid(centroid, transform_listener):
	rest_period = rospy.Duration(0.2)
	world_frame = rospy.get_param("convex_hull/reference_frame")
	kinect_frame = rospy.get_param("convex_hull/perception_link")
	print "kinect_frame: ", kinect_frame, " world_frame: ", world_frame
	while True:
		try:
			(trans,rot) = transform_listener.lookupTransform(world_frame, kinect_frame, rospy.Time(0))
			break
		except:
			print "Caught transform Exception"
			rest_period.sleep()
			continue
	
	print "The kinect is located at: ", trans
	print "The centroid is located at: ", centroid
	
	camera_to_centroid = [c - k for c,k in zip(centroid, trans)]
	print "Camera to centroid: ", camera_to_centroid
	
	return camera_to_centroid


def apply_directional_filter(camera_to_centroid, approach_rays, acceptable_angle):
	out_ray_idxs = []
	for idx, ray in enumerate(approach_rays):
		approach_dir = -ray[3:6]
		if vector_math.angle_between(camera_to_centroid, approach_dir) <= acceptable_angle:
			out_ray_idxs.append(idx)

	return approach_rays.take(out_ray_idxs, axis=0)

def random_selection(approach_vecs, num_rays):
	if num_rays >= len(approach_vecs) or num_rays == 0:
		return approach_vecs
	
	vector_math.np.shuffle(approach_vecs)
	return approach_vecs[:num_rays]
