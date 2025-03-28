/* \author Joel Milla */

#include "cluster.h"
#include "pcl/PointIndices.h"
#include <cstddef>
#include <queue>
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
void EuclideanCluster<PointT>::proximity(pcl::PointIndices &cluster, boost::dynamic_bitset<>& points_processed, int point_indx) const {
	std::queue<int> queue;

	queue.push(point_indx);
	points_processed.set(point_indx);

	//* Iterate over all the nearby points
	while (!queue.empty()) {
		int curr_point_indx = queue.front();
		queue.pop();
		cluster.indices.push_back(curr_point_indx);

		if (cluster.indices.size() > max_cluster_size) {
			cluster.indices = {};
			return;
		}

		PointT curr_point = input_cloud->points[curr_point_indx];
		std::vector<int> nearby_points = tree.search(curr_point);

		//* We iterate over the nearby points, add them to the queue to be processed, and insert them into the processed set in order to not be added twice
		for (const int& point : nearby_points) {
			bool point_processed = points_processed.test(point);
			if (point_processed)
				continue;

			queue.push(point);
			points_processed.set(point);
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
	// std::unordered_set<int> points_processed;
	// std::bitset<input_cloud->points.size()> points_processed(0);
	boost::dynamic_bitset<> points_processed(input_cloud->points.size());

	for (std::size_t point_indx = 0; point_indx < points.size(); point_indx++) {
		bool point_processed = points_processed.test(point_indx);

		if (point_processed)
			continue;
		
		pcl::PointIndices cluster;
		proximity(cluster, points_processed, point_indx);

		if (cluster.indices.size() != 0 && cluster.indices.size() > min_cluster_size)
			cluster_indices.push_back(cluster);
	}
}

/**
	* @brief Sets the value of max_cluster size to limit the search results 
	* 
	* @param max_size maximum size
	* @return template <typename PointT> void 
	*/
template <typename PointT>
void EuclideanCluster<PointT>::setMaxClusterSize(std::size_t max_size) {
	max_cluster_size = max_size;
}

/**
	* @brief Sets the value of min_cluster size to limit the search results 
	* 
	* @param max_size minimum size
	* @return template <typename PointT> void 
	*/
template <typename PointT>
void EuclideanCluster<PointT>::setMinClusterSize(std::size_t min_size) {
	min_cluster_size = min_size;
}