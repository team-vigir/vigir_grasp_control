#include "cluster_segmentation.h"

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ& target_point)
{
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cluster_vector;
  pcl::PCDWriter writer;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (full_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
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

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    std::stringstream ss;
    ss << "/home/eva/Desktop/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

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
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.0125);

  seg.setInputCloud (full_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () < 300)
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
  
  cout << "GUYS!!! WE NEED TO VERIFY THAT THE NORMALS ARE NOT PARALLEL! AND DEAL WITH PLANES THAT DONT INTERSECT" << endl;
  return intersection_line;  
}

void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("/home/eva/Desktop/" + filename + ".pcd", *cloud, false);
}

void save_planes(vector<Plane> all_planes)
{
  std::stringstream ss;
  string filename = "segmented_plane";
  string num;
  for(int i = 0; i < all_planes.size(); ++i) {
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

void remove_largest_plane(vector<Plane>& plane_vector)
{
  int index_tracker = -1;
  int point_tracker = -1;
  int num_pts;
  for(int i = 0; i < plane_vector.size(); ++i) {
    num_pts = plane_vector[i].pts->points.size();
    if(num_pts > point_tracker){
      index_tracker = i;
      point_tracker = plane_vector[i].pts->points.size();
    }
  }
  if(index_tracker == -1 || point_tracker == -1) {
    cout << "Error, could not determine largest plane... now exiting function";
    return;
  }

  plane_vector.erase(plane_vector.begin() + index_tracker);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud_and_planes(vector<Plane>& plane_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  *combined_cloud = *remaining_cloud;
  for(int i = 0; i < plane_vector.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_plane = plane_vector[i].pts;
    *combined_cloud += *cur_plane;
  }
  return combined_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr return_nearest_cluster(pcl::PointXYZ selected_point, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vector) 
{
  double nearest_distance = 99999999999999999;
  double cur_distance;
  int idx;
  for(int i = 0; i < cluster_vector.size(); ++i) {
    if((cur_distance = return_distance_nearest_point(cluster_vector[i], selected_point)) < nearest_distance) {
      nearest_distance = cur_distance;
      idx = i;
    }
  }
  return cluster_vector[idx];
}

double return_distance_nearest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ selected_point)
{
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
    for (int k = 0; k < pointIdxNKNSearch.size (); ++k)
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
pcl::PointCloud<pcl:PointXYZ>::Ptr isolate_hull_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ selected_point) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/eva/Desktop/plane_seg_cloud_2.pcd", *cloud) == -1) //* load the file
    {
      cout << "Couldn't read file test_pcd.pcd \n";
      EXPECT_TRUE(false);
    }

  stat_outlier_remove(full_cloud);

  vector<Plane> all_planes;
  all_planes = find_all_planes(full_cloud);
  remove_largest_plane(all_planes);

  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
  combined_cloud = combine_cloud_and_planes(all_planes, remaining_cloud);

  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separated_clusters;
  separated_clusters = get_clusters(combined_cloud);
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr isolated_cluster;
  isolated_cluster = return_nearest_cluster(selected_point, separated_clusters);
}