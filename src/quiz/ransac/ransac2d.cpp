/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cstdlib>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for (int it = 0; it < maxIterations; it++) {
		std::unordered_set<int> inliersResultTemp;

		// Randomly sample subset and fit line
		long long int sizeCloud = cloud->points.size();
		long long int index1 = rand() % sizeCloud;
		long long int index2 = rand() % sizeCloud;
		long long int index3 = rand() % sizeCloud;

		pcl::PointXYZ point1 = cloud->points[index1];
		pcl::PointXYZ point2 = cloud->points[index2];
		pcl::PointXYZ point3 = cloud->points[index3];

		long long int A = ( (point2.y - point1.y)*(point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y));
		long long int B = ( (point2.z - point1.z)*(point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z));
		long long int C = ( (point2.x - point1.x)*(point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x));
		long long int D = -1 * ((A * point1.x) + (B * point1.y) + (C * point1.z));

		// Measure distance between every point and fitted line
		for (int indx = 0; indx < cloud->points.size(); indx++) {
			const pcl::PointXYZ& point = cloud->points[indx];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			double distance = abs( (A * x) + (B * y) + (C * z) + (D) ) / sqrt( (A * A) + (B * B) + (C * C));
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
				inliersResultTemp.insert(indx);
			
			// If is maximum inliers that ever existed, save result
			if (inliersResult.size() < inliersResultTemp.size())
				inliersResult = inliersResultTemp;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// randombly speaking, the more % you have of outliers, the more chance the model has on finding the line. Because can use inliers to fit the model. However, if have low % of inliers, will need more iteartions to find the line
	// the distance threshold is related to noise, the more noise you have the higher a distance threshold. because if have noise, then it means that your actual inliers will be more disperse and in general values will be more disperesed, so you want them to be included.
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
