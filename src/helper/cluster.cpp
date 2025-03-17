/* \author Joel Milla */

#include "cluster.h"
#include "pcl/PointIndices.h"
#include <queue>
#include <unordered_set>
#include <vector>

template <typename PointT>
EuclideanCluster<PointT>::EuclideanCluster() {}

template <typename PointT>
EuclideanCluster<PointT>::~EuclideanCluster() {}

/*
Input:
@input_cloud => pointer to the orignial point cloud data

Function:
Uses the instance of tree to populate the tree
*/
template<typename PointT>
void EuclideanCluster<PointT>::setInputCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud, float distance_tol) {
	if (!input_cloud || input_cloud->points.empty()) {
        throw std::invalid_argument("Input cloud is null or empty");
    }
    if (distance_tol <= 0) {
        throw std::invalid_argument("Distance tolerance must be positive");
    }

	this->input_cloud = input_cloud;
	tree.setTree(input_cloud);
	tree.setDistanceTol(distance_tol);
}

/*
Input:
@cluster => cluster where the indices will be added
@points_processed => set that will keep track of the points that have been added to the cluster
@point_indx => initial point indx that we want to obtain nearby points

Return:
void

Function:
Recieves a reference cluster, and the function adds to that cluster all the nearby points. 
*/
template <typename PointT>
void EuclideanCluster<PointT>::proximity(pcl::PointIndices &cluster, std::unordered_set<int>& points_processed, int point_indx) const {
	std::queue<int> queue;

	queue.push(point_indx);
	points_processed.insert(point_indx);

	//* Iterate over all the nearby points
	while (!queue.empty()) {
		int curr_point_indx = queue.front();
		queue.pop();
		cluster.indices.push_back(curr_point_indx);

		PointT curr_point = input_cloud->points[curr_point_indx];
		std::vector<int> nearby_points = tree.search(curr_point);

		//* We iterate over the nearby points, add them to the queue to be processed, and insert them into the processed set in order to not be added twice
		for (const int& point : nearby_points) {
			bool point_processed = points_processed.find(point) != points_processed.end();
			if (point_processed)
				continue;

			queue.push(point);
			points_processed.insert(point);
		}
	}
}

/*
Input
@cluster_indicies => vector of pointIndices that will be populated by clusters

Function:
Receiving cluster_indicies, the function will traverse all the points and divide the points in different clusters (which are vector of indices)
*/
template <typename PointT>
void EuclideanCluster<PointT>::euclideanCluster(std::vector<pcl::PointIndices> &cluster_indices) const {

	const auto &points = input_cloud->points;
	std::unordered_set<int> points_processed;

	for (int point_indx = 0; point_indx < points.size(); point_indx++) {
		bool point_processed = points_processed.find(point_indx) != points_processed.end();

		if (point_processed)
			continue;
		
		pcl::PointIndices cluster;
		proximity(cluster, points_processed, point_indx);
		cluster_indices.push_back(cluster);
	}
}