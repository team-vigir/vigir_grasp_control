#include "cluster_segmentation.h"

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, int min_cluster_size)
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vector;
	pcl::PCDWriter writer;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (full_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	//int min_cluster_size = 1; ADDED THIS TO ALLOW KINECT CALIBRATION TO PROPERLY WORK (original value was 50). may want to consider adding as a parameter
	ec.setMinClusterSize (min_cluster_size);
	ec.setMaxClusterSize (2500000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (tree->getInputCloud());
	ec.extract (cluster_indices);

	int j = 0;
	int num_clusters = cluster_indices.size();
	for (int i = 0; i < num_clusters; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = extract_subcloud(full_cloud, cluster_indices[i]);

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		/*std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "/home/eva/Desktop/cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
*/
		cluster_vector.push_back(cloud_cluster);

		j++;
	}

	return cluster_vector;
}

vector<Plane> find_all_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr &full_cloud)
{
	//pcl::PCDWriter writer;
	vector<Plane> all_planes;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	
	Plane cur_plane;
	pcl::PointIndices::Ptr inliers;
	while((inliers = planar_segmentation(full_cloud, cur_plane)) && inliers->indices.size()){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p( new pcl::PointCloud<pcl::PointXYZ>);

		extract.setInputCloud(full_cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud_f);
		extract.setNegative(false);
		extract.filter(*cloud_p);

		full_cloud.swap(cloud_f);
		cur_plane.pts = cloud_p;
		all_planes.push_back(cur_plane);
	}
	
	ROS_INFO_STREAM("\tFiltered " << all_planes.size()  << " planes.");
	
	// pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud = extract_subcloud(full_cloud, *inliers);
	//writer.write<pcl::PointXYZ> ("/home/eva/Desktop/plane_cluster1.pcd", *plane_cloud, false);
	return all_planes;
}

pcl::PointIndices::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, Plane& new_plane) 
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.015);

	seg.setInputCloud (full_cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () < 1500)
	{

		inliers->indices.clear();
		return inliers;
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	new_plane.coef = coefficients;

	return inliers;

}

//May have to change this back to original copied text for get_clusters to work
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_subcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices& indices) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	out_cloud->header = full_cloud->header;
	//  out_cloud->height = full_cloud->height;
	out_cloud->is_dense = full_cloud->is_dense;
	for (std::vector<int>::const_iterator pit = indices.indices.begin (); pit != indices.indices.end (); pit++){
		out_cloud->push_back(full_cloud->points[*pit]); //*
	}

	return out_cloud;
}

//FORREST AND JOSH'S STUFFS (jackson too)!

Line plane_intersect(pcl::ModelCoefficients::Ptr plane1, pcl::ModelCoefficients::Ptr plane2)
{
	Eigen::Vector3d normalPlane1 = get_normal(plane1);
	Eigen::Vector3d normalPlane2 = get_normal(plane2);
	double dist;
	pcl::PointXYZ point = init_pt(0, 0, 0);

	Eigen::Vector3d slope = normalPlane1.cross(normalPlane2);

	normalize_plane(plane1);
	dist = pt_to_plane_dist(plane1, point);

	normalPlane1.normalize();
	Eigen::Vector3d point_on_plane = init_vec(point) - normalPlane1*dist;
	cout << "Point on Plane: " << point_on_plane << endl;
	Eigen::Vector3d projecting_slope = slope.cross(normalPlane1);
	cout << "Projecting Slope: " << projecting_slope << endl;

	Line projecting_line(projecting_slope, point_on_plane);
	Eigen::Vector3d pt_on_intersecting_line = projecting_line.find_plane_intersection(plane2);

	Line intersection_line(slope, pt_on_intersecting_line);

	cout << "GUYS!!! WE NEED TO VERIFY THAT THE NORMALS ARE NOT PARALLEL! AND DEAL WITH PLANES THAT DONT INTERSECT" << endl;
	return intersection_line;  
}

void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename)
{
	pcl::PCDWriter writer;
	string hullify_path = ros::package::getPath("hullify");
	writer.write<pcl::PointXYZ> (hullify_path + "/clouds/" + filename + ".pcd", *cloud, false);
}

void save_planes(vector<Plane> all_planes)
{
	std::stringstream ss;
	string filename = "segmented_plane";
	string num;
	for(unsigned int i = 0; i < all_planes.size(); ++i) {
		ss << i;
		num = ss.str();
		string temp = filename + num;
		save_cloud(all_planes[i].pts, temp);
	}
}

void stat_outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (20);
	sor.setStddevMulThresh (1);
	sor.filter (*cloud_filtered);

	cloud = cloud_filtered;
}

 pcl::PointCloud<pcl::PointXYZ>::Ptr remove_largest_plane(vector<Plane>& plane_vector)
{
	int index_tracker = -1;
	int point_tracker = -1;
	int num_pts;
	for(unsigned int i = 0; i < plane_vector.size(); ++i) {
		num_pts = plane_vector[i].pts->points.size();
		if(num_pts > point_tracker && num_pts > 500){
			index_tracker = i;
			point_tracker = plane_vector[i].pts->points.size();
		}
	}
	if(index_tracker == -1 || point_tracker == -1) {
		cout << "\tError, could not determine largest plane... now exiting function" << endl;
		return  pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pts = plane_vector[index_tracker].pts;
	plane_vector.erase(plane_vector.begin() + index_tracker);

	return plane_pts;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud_and_planes(vector<Plane>& plane_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	*combined_cloud += *remaining_cloud;
	for(unsigned int i = 0; i < plane_vector.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cur_plane = plane_vector[i].pts;
		*combined_cloud += *cur_plane;
	}
	return combined_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr return_nearest_cluster(pcl::PointXYZ selected_point, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_vector) 
{
	double nearest_distance = 99999999999999999;
	double cur_distance;
	int idx = -1;
	for(unsigned int i = 0; i < cluster_vector.size(); ++i) {
		if((cur_distance = return_distance_nearest_point(cluster_vector[i], selected_point)) < nearest_distance) {
			nearest_distance = cur_distance;
			idx = i;
		}
	}

	return cluster_vector[idx];
}

double return_distance_nearest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ selected_point)
{
	if (cloud->points.size() < 10){
		return 100000;
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(cloud);

	int numNeighbors = 1;

	std::vector<int> pointIdxNKNSearch(numNeighbors);
	std::vector<float> pointNKNSquaredDistance(numNeighbors);

	std::cout << "K nearest neighbor search at (" << selected_point.x 
		<< " " << selected_point.y 
		<< " " << selected_point.z
		<< ") with K=" << numNeighbors << std::endl;

	if ( kdtree.nearestKSearch (selected_point, numNeighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	{
		for (unsigned int k = 0; k < pointIdxNKNSearch.size (); ++k)
			std::cout << "    "  <<   cloud->points[pointIdxNKNSearch[k]].x 
				<< " " << cloud->points[pointIdxNKNSearch[k]].y 
				<< " " << cloud->points[pointIdxNKNSearch[k]].z 
				<< " (squared distance: " << pointNKNSquaredDistance[k] << ")" << std::endl;
	}
	return pointNKNSquaredDistance[0];
}

//need to change PCD file load to a more flexible path or ask for user designated path
//should consider putting limits on "remove_largest_plane()" so that it does not remove the target object
/*ex: in "remove_largest_plane()", if "return_distance_nearest_point()" returns a point on the largest
  plane, then do not remove the plane
  */
//NEED TO FINISH THIS FUNCTION!!!
pcl::PointCloud<pcl::PointXYZ>::Ptr isolate_hull_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ selected_point) {
//	pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr remainder;
	//string filename;
	string selection;

	// cout << "Select 1 to isolate point cloud data from hullify, 2 to isolate other point cloud data, or \"exit\" to exit..." << endl;
	// cin >> selection;
	// if(selection != "1" || selection != "2" || selection != "exit") {
	// 	cout << "Unrecognized input. Exiting... ";
	// 	exit;
	// }
	// if(selection == "exit") {
	// 	cout << "Exiting..." << endl;
	// 	exit;
	// }
	// if(selection == "1") {
	// 	cout << "Hullify pipeline hasn't been implented yet. We'll fix that soon! Exiting..." << endl;
	// 	exit;
	// }
	// if(selection == "2") {
	// 	cout << "Please provide the path and filename for the point cloud: " << endl;
	// 	cin >> filename;
	/*if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/sample_full_cloud_1.pcd", *full_cloud) == -1) // load the file. was previously 
	{
		cout << "Couldn't read file test_pcd.pcd \n";
		exit(1);
	}*/

	stat_outlier_remove(full_cloud);
	
	cout << "\tCloud size after stat outlier removal: " << full_cloud->points.size() << endl;
	vector<Plane> all_planes;
	all_planes = find_all_planes(full_cloud);
	remainder = full_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr largest_plane = remove_largest_plane(all_planes);
	
	cout << "\tRemainder has " << remainder->points.size() << " points." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
	combined_cloud = combine_cloud_and_planes(all_planes, remainder);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separated_clusters;
	separated_clusters = get_clusters(combined_cloud);

	cout << "\tNum clusters (no plane): " << separated_clusters.size() << endl;
	separated_clusters.push_back(largest_plane);
	pcl::PointCloud<pcl::PointXYZ>::Ptr isolated_cluster;
	
	if (separated_clusters.size() == 1 && largest_plane->size() < 50) {
		ROS_ERROR("No convex hull can be perceived from pointcloud. How is the density of the cloud?");
		throw string("No hull.");
	} else {
		isolated_cluster = return_nearest_cluster(selected_point, separated_clusters);
	}

/*string input;
	cout << "How does that look?" << endl;
	cin >> input;
*/
	return isolated_cluster;
}
