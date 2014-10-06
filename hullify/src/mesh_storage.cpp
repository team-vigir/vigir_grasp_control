#include "mesh_storage.h"

MeshStorage::MeshStorage()
{
	//Register the service callback
	dist_service = nh.advertiseService("CheckGraspDistance", &MeshStorage::check_grasp_distance, this);
}

void MeshStorage::set_mesh(pcl::PolygonMesh::Ptr mesh)
{
	this->mesh = mesh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	fromPCLPointCloud2(mesh->cloud, *cloud);
	mesh_tree.setInputCloud(cloud);
}

bool MeshStorage::check_grasp_distance(osu_grasp_msgs::CheckGraspDistance::Request& req, osu_grasp_msgs::CheckGraspDistance::Response& resp)
{
	pcl::PointXYZ wrist_center_pt;
	wrist_center_pt.x = req.wrist_center_location.x;
	wrist_center_pt.y = req.wrist_center_location.y;
	wrist_center_pt.z = req.wrist_center_location.z;

	double nearest_distance;
	try {
		pcl::PointXYZ nearest_neighbor = get_nearest_neighbor(wrist_center_pt, nearest_distance);
	
	} catch (exception& e) {
		cout << e.what() << endl;
		resp.within_acceptable_dist = false;
		return false;
	}

	if (nearest_distance <= req.acceptable_dist){
		resp.within_acceptable_dist = true;
	} else {
		resp.within_acceptable_dist = false;
	}

	return true;
}

pcl::PointXYZ MeshStorage::get_nearest_neighbor(pcl::PointXYZ point, double& distance)
{
	vector<int> nearest_neighbor_idx(1);
	vector<float> nearest_neighbor_dist_sq(1);

	if ( mesh_tree.nearestKSearch (point, 1, nearest_neighbor_idx, nearest_neighbor_dist_sq) > 0 ){
		distance = sqrt(nearest_neighbor_dist_sq[0]);
		return mesh_tree.getInputCloud()->points[nearest_neighbor_idx[0]];

	} else {
		no_search_results excp;
		excp.ptcloud_size = mesh_tree.getInputCloud()->points.size();
		excp.pt = point;
		throw no_search_results();
	}
}