/* \author Joel Milla */

#include "cluster.h"

// Helper function receives the target point, and saves all the nearby points of the target into the cluster
template <typename PointT>
void Cluster::helperProximty(int indx, std::vector<int>& cluster, std::unordered_set<int>& pointsProcessed, KdTreeSpace::KdTree<PointT>* tree, float distanceTol, const std::vector<std::vector<float>>& points) {

	pointsProcessed.insert(indx); // mark the point received as processed
	cluster.push_back(indx); // save the current point into the cluster

	// Find nearby points of the target
	std::vector<int> nearbyPoints = tree->search(points[indx], distanceTol);

	// Traverse the indices of the closest points
	for (int newTargetIndx : nearbyPoints) {

		bool pointProcessed = pointsProcessed.find(newTargetIndx) != pointsProcessed.end();

		if (pointProcessed)
			continue;

		helperProximty(newTargetIndx, cluster, pointsProcessed, tree, distanceTol, points);
	}
}

template <typename PointT>
std::vector<std::vector<int>> Cluster::euclideanCluster(const std::vector<std::vector<float>>& points, KdTreeSpace::KdTree<PointT>* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::unordered_set<int> pointsProcessed;

	for (int indx = 0; indx < points.size(); indx++) {
		const std::vector<float>& target = points[indx];
		bool pointProcessed = pointsProcessed.find(indx) != pointsProcessed.end();

		if (pointProcessed)
			continue;
		
		std::vector<int> cluster;
		helperProximty(indx, cluster, pointsProcessed, tree, distanceTol, points); // call recursively helperProximity to find closest points relative to the target

		clusters.push_back(cluster);
	}

	return clusters;

}