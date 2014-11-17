#include "cluster_segmentation.h"

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud)
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vector;
	pcl::PCDWriter writer;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (full_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.03); // 2cm
	ec.setMinClusterSize (50);
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

	// pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud = extract_subcloud(full_cloud, *inliers);
	//writer.write<pcl::PointXYZ> ("/home/eva/Desktop/plane_cluster1.pcd", *plane_cloud, false);
	return all_planes;
}

pcl::PointIndices::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, Plane& new_plane) 
{
	if (full_cloud->size() == 0){
		cout << "No more planes to be had!" << endl;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		inliers->indices.clear();
		return inliers;
	}
	
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
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
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

	cout << "GUYS!!! " << endl;//WE NEED TO VERIFY THAT THE NORMALS ARE NOT PARALLEL! AND DEAL WITH PLANES THAT DONT INTERSECT" << endl;
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
		cout << "Error, could not determine largest plane... now exiting function";
		return  pcl::PointCloud<pcl::PointXYZ>::Ptr(new  pcl::PointCloud<pcl::PointXYZ>);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pts = plane_vector[index_tracker].pts;
	plane_vector.erase(plane_vector.begin() + index_tracker);

	return plane_pts;
}

void remove_table_planes(vector<Plane>& plane_vector, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& removed_planes)
{
	unsigned int num_planes = plane_vector.size();
	Eigen::VectorXf current_line;
	vector <int> inliers;
	for (unsigned int i = 0; i < num_planes; ++i){
		//Identify the Largest two principle axes given the almost-planar points
		pcl::PCA<pcl::PointXYZ> component_finder;
		component_finder.setInputCloud(plane_vector[i].pts);
		Eigen::Matrix3f principal_axes_float = component_finder.getEigenVectors();
		Eigen::Vector3f eigenvalues = component_finder.getEigenValues();
		Eigen::Matrix3d principal_axes = principal_axes_float.cast<double>();

		//Eigen::Vector3d pca1 = principal_axes.col(0);
		//Eigen::Vector3d pca2 = principal_axes.col(1);

		if (eigenvalues[1] > 4.0){
			if (eigenvalues[0] < (3 * eigenvalues[1])){
				//This plane is probably a table or wall, its smaller "side" is more than half the bigger one.
				ROS_INFO("Removing a table-like flat surface based on its dimensions.");
				removed_planes.push_back(plane_vector[i].pts);
				cout << "i: " << i << " size: " << plane_vector.size() << endl;
				plane_vector.erase(plane_vector.begin() + i);
				i--;
				num_planes--;
			}
		}

		//ROS_INFO_STREAM("For plane " << i << " the largest principal component e_val is " << endl << eigenvalues[0] << endl << " and the second is: " << endl << eigenvalues[1] << endl);
	}
}

/* This idea didn't work...
		//Fit a linear model to the plane
		vector<int> indices = get_all_indices(plane_vector[i].pts);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr planar_copy (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr plane_proj_pts = project_pts_plane(plane_vector[i].pts, plane_vector[i].coef);
		/planar_copy = *plane_proj_pts;
		cout << "Plane_proj_pts size: " << plane_proj_pts->size() << endl;
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model (new pcl::SampleConsensusModelLine<pcl::PointXYZ>(plane_proj_pts, indices));;
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (line_model);
		ransac.setDistanceThreshold(0.15);
		ransac.computeModel();
		ransac.getModelCoefficients(current_line);

		cout << "Line: " << endl << current_line << endl;

		//Specify an accuracy threshold of the projection?
		variance = line_model.computeVariance();
		cout << "Variance in linear model: " << variance << endl;
		if (variance > 1000){
			//Not well approximated by a line...
		}

		//Push the determined line out to the edges of the object


		pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj_pts (new pcl::PointCloud<pcl::PointXYZ>);
		line_model->projectPoints(inliers, current_line, *line_proj_pts, false); 

		double width = get_width(line_model, current_line, plane_proj_pts, line_proj_pts);
		
		//Use projected points to find farthest co-linear projection.
		double height = get_height(current_line, line_proj_pts);

		//Specify dimensions to remove
		if (width > 0.14 && height > 0.14){
			ROS_INFO_STREAM("Removed a table-like plane with height: " << height << " and width: " << width);
			removed_planes.push_back(plane_vector[i].pts);
			plane_vector.erase(plane_vector.begin() + i);
			i--;
		}

*/
/*
vector<int> get_all_indices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	vector<int> inliers;
	inliers.clear();
	int num_pts = cloud->points.size();
	for (int j = 0; j < num_pts; ++j){
		inliers.push_back(j);
	}

	return inliers;
}

double get_height(Eigen::VectorXf current_line, pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj_pts)
{
	double slope_x = current_line[3];
	int max_inline_idx = 0;
	double max_inline_t = 0;
	int max_outline_idx = 0;
	double max_outline_t = 0;
	
	double t;
	unsigned int num_pts = line_proj_pts->size();
	for (unsigned int j = 0; j < num_pts; ++j){
		if (current_line[3] != 0){
			t = ((*line_proj_pts)[j].x - (*line_proj_pts)[j].x) / slope_x;
		} else if (current_line[4] != 0){
			t = ((*line_proj_pts)[j].y - (*line_proj_pts)[j].y) / current_line[4];
		} else {
			t = ((*line_proj_pts)[j].z - (*line_proj_pts)[j].z) / current_line[5];
		}

		if (t > 0){
			if (t > max_inline_t){
				max_inline_idx = j;
				max_inline_t = t;
			}
		} else {
			if (t < max_outline_t){
				max_outline_idx = j;
				max_outline_t = t;
			}
		}
	}

	double height = pt_dist((*line_proj_pts)[max_inline_idx], (*line_proj_pts)[max_outline_idx]);
	ROS_INFO_STREAM("Height for plane model: " << height);

	return height;
}

double get_width(pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model, Eigen::VectorXf current_line, pcl::PointCloud<pcl::PointXYZ>::Ptr pts, pcl::PointCloud<pcl::PointXYZ>::Ptr line_proj_pts)
{
	vector<double> proj_distances;
	line_model->getDistancesToModel(current_line, proj_distances);

	cout << "cloud1 location: " << (void*) &(*pts) << " cloud2 location: " << (void*) &(*line_proj_pts) << endl;

	double max_inline_idx = 0;
	double max_inline_dist = 0;
	double max_outline_idx = 0;
	double max_outline_dist = 0;
	Eigen::Vector3d reference_vector = init_vec((*pts)[0]) - init_vec((*line_proj_pts)[0]);
	Eigen::Vector3d cur_vec;
	unsigned int num_pts = pts->size();
	for (unsigned int i = 0; i < num_pts; ++i){
		cur_vec = init_vec((*pts)[i]) - init_vec((*line_proj_pts)[i]);
		cout << "Cur vec: " << (*pts)[i] << endl << "reference vector: " << (*line_proj_pts)[i] << endl;
		string input;
		cin >> input;
		if (get_angle_mag_between(reference_vector, cur_vec) < (M_PI/2)){
			if (proj_distances[i] > max_inline_dist){
				max_inline_idx = i;
				max_inline_dist = proj_distances[i];
			}
		} else {
			if (proj_distances[i] > max_outline_dist){
				max_outline_idx = i;
				max_outline_dist = proj_distances[i];
			}
		}
	}

	if (max_inline_dist == 0 || max_outline_dist == 0){
		ROS_ERROR("In cluster_segmentation/get_width(), one of the distances for plane removal is not defined.");
		exit(1);
	}

	double width = sqrt(max_inline_dist) + sqrt(max_outline_dist);
	ROS_INFO_STREAM("Width of model: " << width);
	return width;
}
*/
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

//THIS IS THE ONE FUNCITON TO RULE THEM ALL (UNTESTED CURRENTLY)
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

	cout << "Cloud size after stat outlier removal: " << full_cloud->points.size() << endl;
	vector<Plane> all_planes;
	all_planes = find_all_planes(full_cloud);
	remainder = full_cloud;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> removed_clouds;
	remove_table_planes(all_planes, removed_clouds);

	cout << "Remainder has " << remainder->points.size() << " points." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
	combined_cloud = combine_cloud_and_planes(all_planes, remainder);

	cout << "Combined cloud has " << combined_cloud->points.size() << " points." << endl;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separated_clusters;
	separated_clusters = get_clusters(combined_cloud);

	cout << "Num clusters: " << separated_clusters.size() << endl;
	//separated_clusters.push_back(largest_plane);
	//separated_clusters.insert(separated_clusters.begin(), removed_clouds.begin(), removed_clouds.end());
	pcl::PointCloud<pcl::PointXYZ>::Ptr isolated_cluster;
	if (separated_clusters.size() == 0){
		ROS_INFO("No graspable cluster found in cloud.");
		return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	}

	isolated_cluster = return_nearest_cluster(selected_point, separated_clusters);

	/*string input;
	  cout << "How does that look?" << endl;
	  ciseparated_clusters.sizen >> input;
	 */
	return isolated_cluster;
}
