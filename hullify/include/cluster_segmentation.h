#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <iostream>

#include "plane_reps_and_3dmath.h"

using std::cout;
using std::cin;
using std::endl;

struct Plane{
	pcl::ModelCoefficients::Ptr coef;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts;

};

void stat_outlier_remove(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > get_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud);
vector<Plane> find_all_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr &full_cloud);
pcl::PointIndices::Ptr planar_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, Plane& new_plane);

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_subcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, const pcl::PointIndices& indices);

Line plane_intersect(pcl::ModelCoefficients::Ptr plane1, pcl::ModelCoefficients::Ptr plane2);

void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename);
void save_planes(vector<Plane> all_planes);

void remove_largest_plane(vector<Plane>& plane_vector);
pcl::PointCloud<pcl::PointXYZ>::Ptr combine_cloud_and_planes(vector<Plane>& plane_vector, pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr return_nearest_cluster(pcl::PointXYZ selected_point, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_vector);
double return_distance_nearest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ selected_point);

//DUN DUN DUNNNNNNNNNNNNNN
pcl::PointCloud<pcl::PointXYZ>::Ptr isolate_hull_cluster(pcl::PointXYZ selected_point);
