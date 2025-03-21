#include "segment.h"
#include <cstddef>

template<typename PointT>
Segment<PointT>::Segment() {}

template<typename PointT>
Segment<PointT>::~Segment() {}

/*
Params:
@inliers -> indices where the plane found will be saved
@cloud -> cloud to be filtered
@maxIterations -> max iterations to be implemented during RANSAC
@distanceTol -> distance tolerance between points

Return:
Nothing

Function:
Implements RANSAC algorithm to separate the plane and the objects from the cloud. Run maxIterations and inlcude as inliers the points that are within the distanceTol from the main plain. The inliers are the road. Also, segmentation uses iterative approach, the more iterations the more confident, but also takes longer. Algirthm fits plane to a point and uses distanceTolerance to decide which points belong to the plane.  
*/
template<typename PointT>
void Segment<PointT>::Ransac(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    std::vector<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for (int it = 0; it < maxIterations; it++) {
		std::vector<int> inliersResultTemp;

		// Randomly sample subset and fit line
		long long int sizeCloud = cloud->points.size();
		long long int index1 = rand() % sizeCloud;
		long long int index2 = rand() % sizeCloud;
		long long int index3 = rand() % sizeCloud;

		PointT point1 = cloud->points[index1];
		PointT point2 = cloud->points[index2];
		PointT point3 = cloud->points[index3];

		long long int A = ( (point2.y - point1.y)*(point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y));
		long long int B = ( (point2.z - point1.z)*(point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z));
		long long int C = ( (point2.x - point1.x)*(point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x));
		long long int D = -1 * ((A * point1.x) + (B * point1.y) + (C * point1.z));

		// Measure distance between every point and fit line
		for (std::size_t indx = 0; indx < cloud->points.size(); indx++) {
			const PointT& point = cloud->points[indx];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			double distance = abs( (A * x) + (B * y) + (C * z) + (D) ) / sqrt( (A * A) + (B * B) + (C * C));
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
				inliersResultTemp.push_back(indx);
			
			// If is maximum inliers that ever existed, save result
			if (inliersResult.size() < inliersResultTemp.size())
				inliersResult = inliersResultTemp;
		}
	}

	// Save indices of the plane in inliers
	inliers->indices = inliersResult;
}