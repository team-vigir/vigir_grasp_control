#include "cluster_segmentation.h"

void get_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ& target_point)
{
  pcl::PCDWriter writer;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (full_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
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
    ss << "/home/atlas/Desktop/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
}

void planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud) 
{
  pcl::PCDWriter writer;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  // Optional
  //seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.0075);

  seg.setInputCloud (full_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud = extract_subcloud(full_cloud, *inliers);
  writer.write<pcl::PointXYZ> ("/home/atlas/Desktop/plane_cluster.pcd", *plane_cloud, false); 
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_subcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices& indices) 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = indices.indices.begin (); pit != indices.indices.end (); pit++){
    out_cloud->points.push_back (full_cloud->points[*pit]); //*
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