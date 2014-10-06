//ros includes
#include "ros/ros.h"
#include <osu_grasp_msgs/Mesh_and_bounds.h>
#include "osu_grasp_msgs/CheckGraspDistance.h"

#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <exception>
#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;
using std::exception;

class MeshStorage {
public:
	MeshStorage();

	bool check_grasp_distance(osu_grasp_msgs::CheckGraspDistance::Request& req, osu_grasp_msgs::CheckGraspDistance::Response& resp);
	void set_mesh(pcl::PolygonMesh::Ptr mesh);

	pcl::PointXYZ get_nearest_neighbor(pcl::PointXYZ point, double& distance);


private:
	pcl::PolygonMesh::Ptr mesh;
	pcl::KdTreeFLANN<pcl::PointXYZ> mesh_tree;
	ros::NodeHandle nh;
	ros::ServiceServer dist_service;
};

class no_search_results : public exception{
	virtual const char* what(){
		return "Could not find a nearest neighbor. Is pointcloud empty?";
	}

public:
	int ptcloud_size;
	pcl::PointXYZ pt;
};