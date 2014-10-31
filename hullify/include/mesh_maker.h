#ifndef MESH_MAKER_H
#define MESH_MAKER_H
//ros includes
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <osu_grasp_msgs/Mesh_and_bounds.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include "pcl/ros/conversions.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>

//From Tutorial
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
//End tutorial

#include <Eigen/Dense>

#include "qhull_interface.h"
#include "mesh_bound.h"
#include "hullify_view.h"
#include "pose_transform.h"
#include "cluster_segmentation.h"
#include "mesh_storage.h"
#include "ocs_listener.h"

#include <ctime>
#include <cstdlib>
#include <string>
#include <vector>
using std::string;
using std::vector;
using std::cout;
using std::cin;
using std::endl;

class MeshMaker{
	public:
		MeshMaker();
		~MeshMaker();
		void listen();
		void begin(const std_msgs::String::ConstPtr& msg);
		pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	private:
		void init_reference_frame();
		void init_input_topic();
		void init_mesh_name();
		void init_mesh_ref_frame();
		void convert_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
		sensor_msgs::PointCloud2 transform_ptcloud(const sensor_msgs::PointCloud2& in_cloud, const string& target_frame);
		void set_arm_param(char arm);
		bool get_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr intermediate_cloud);
		bool is_valid_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		geometry_msgs::PoseStamped get_wrist_orientation(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question);
		Axes get_goal_axes(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question);
		Eigen::Matrix3d get_principal_axes(pcl::PointCloud<pcl::PointXYZ>::Ptr pts_in_question);
		void send_hull_and_planes_to_openrave(string& mesh_full_abs_path, pcl::PolygonMesh::Ptr convex_hull, pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts);
		bool are_planes_obtuse(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2);
		void set_bounding_planes(osu_grasp_msgs::Mesh_and_bounds& openrave_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts);
		Eigen::Vector3d get_ninety_degree_base(pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_pts);
		Eigen::Vector3d get_zero_degree_normal(Eigen::Vector3d& horiz_normal, Eigen::Vector3d& camera_to_centroid);
		//void record_planes(hullify::Mesh_and_bounds& msg, Eigen::Vector3d& know_p_proper, Eigen::Vector3d& know_p_improper, Eigen::Vector3d& ninety_normal, Eigen::Vector3d& zero_normal);
		//void set_openrave_msg_planes(hullify::Mesh_and_bounds& msg, Eigen::Vector3d strict_vec1, Eigen::Vector3d strict_vec2, Eigen::Vector3d relaxed_vec1, Eigen::Vector3d relaxed_vec2);
		void mk_mesh_msg(shape_msgs::Mesh& msg, pcl::PolygonMesh::Ptr convex_hull);
		//tf::StampedTransform get_pelvis_transform(string original_frame);

		ros::NodeHandle n;
		tf::TransformListener listener;
		ros::Subscriber grasp_pipeline_trigger;
		vector<ros::Subscriber> cloud_input;
		string visualization_ref_frame;
		string mesh_ref_frame;
		vector<string> in_topic_name;
		string mesh_base_name;
		bool using_left_hand;
		//vector<visualization_msgs::Marker> markers;
		Qhull_int qhull;
		MeshBound* bounds;
		Hullify_View* view;
		MeshStorage* storage;
		Ocs_listener* ocs_contact;
};

//pcl::PolygonMesh::Ptr mk_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
double pt_dist(pcl::PointXYZ pt1, pcl::PointXYZ pt2);
#endif
