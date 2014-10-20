#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>

#include "plane_reps_and_3dmath.h"

using std::cout;
using std::cin;
using std::endl;

void get_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointXYZ& target_point);

void planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_subcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices& indices);

//JOSH AND FORREST'S STUFF (jackson too)!

void plane_intersect(pcl::ModelCoefficients::Ptr plane1, pcl::ModelCoefficients::Ptr plane2, Eigen::Vector3d& slope, Eigen::Vector3d& intersect);