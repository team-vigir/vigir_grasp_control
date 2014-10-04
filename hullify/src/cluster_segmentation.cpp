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

void plane_intersect(pcl::ModelCoefficients::Ptr plane1, pcl::ModelCoefficients::Ptr plane2, Eigen::Vector3d& slope, Eigen::Vector3d& intersect)
{
  pcl::ModelCoefficients::Ptr unitNormalPlane1 = get_unit_normal(plane1);
  pcl::ModelCoefficients::Ptr unitNormalPlane2 = get_unit_normal(plane2);

  //double planeCoefficients1[4], planeCoefficients2[4];
  Eigen::MatrixXd A(2, 2), B(2, 1); //assume z = 0 !!may be an issue with planes that dont actually cross the z axis
  
  slope = unitNormalPlane1.cross(unitNormalPlane2);

  A << plane1->values[0], plane1->values[1],
       plane2->values[0], plane2->values[1];
  B << plane1->values[3],
       plane2->values[3];

  

  /*
  planeCoefficients1 = {plane1->values[0], plane1->values[1], 0, plane1->values[3]}; //3rd component "z" set to 0 in order to find point on line of intersection
  planeCoefficients2 = {plane2->values[0], plane2->values[1], 0, plane2->values[3]}; //3rd component "z" set to 0 in order to find point on line of intersection
  */

}