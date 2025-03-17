// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
// #include "./helper/segment.h"
#include "./helper/segment.cpp"
#include "./helper/cluster.h"   
#include "./helper/cluster.cpp"
/*
Why need to include the segment.cpp and how does this not throw an error because of multiple declaration of same cpp file? 

When the compiler is making the .o files, if there is a class template, it needs to see the definition (and when is not template, it does not, the declaration is enough). But, the definition of the class and linking stage happens until later during linking stage, at that point this specific class has no way of knowing the definition that is happening in another file because the linking stage has not happened. So, you need to import the cpp file so the compiler is able to see the definition of the file during compilation stage. and the compiler and linker work together so the same template is not declared twice, so as to avoid compilation errors.
*/

constexpr bool OUTPUT_LOGS = false;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    if (OUTPUT_LOGS)
        std::cout << cloud->points.size() << std::endl;
}

/*
Params:
@cloud -> cloud to be filtered that has all the points

Returns
@filtered_cloud -> pair with the cloud of points of rooftop and also its indices

Function that returns both the indices and the cloud cluster of the rooftop
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr> ProcessPointClouds<PointT>::ObtainRoofPoints (typename pcl::PointCloud<PointT>::Ptr cloud) {
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    //* CROPBOX DEFINITION
    // Define the limits of the cropbox
    Eigen::Vector4f minPoint = Eigen::Vector4f (-1.5, -1.7, -1, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f (2.6, 1.7, -0.4, 1);

    pcl::CropBox<PointT> cropBoxFilter (true);
    cropBoxFilter.setInputCloud (cloud);

    // Cropbox slighlty bigger then bounding box of points
    std::vector<int> indices;
    cropBoxFilter.setMin (minPoint);
    cropBoxFilter.setMax (maxPoint);

    // Indices to remove
    cropBoxFilter.filter (indices);
    cropBoxFilter.filter (*cloud_filtered);

    inliers->indices = indices;

    return {cloud_filtered, inliers};
}

/*
Params
@cloud -> cloud to be filtered that has all the points

Returns
@filtered_cloud -> cloud without rooftop points

Function to remove the lidar points that collide with the rooftop. And return the new cloud
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::RemoveRoofPoints (typename pcl::PointCloud<PointT>::Ptr cloud) {
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    
    //* FILTERING OF INDICES
    std::pair<typename pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr> roof = ObtainRoofPoints(cloud);
    pcl::PointIndices::Ptr inliers = roof.second;

    pcl::ExtractIndices<PointT> filter;
    filter.setInputCloud (cloud);
    filter.setIndices (inliers);

    // extract all the cloud points that are not within the cropbox
    filter.setNegative (true);
    filter.filter (*cloud_filtered);

    return cloud_filtered;
}

/*
Params:
@cloud -> cloud to be filtered
@filterRes -> value used for voxel grid method
@minPoint -> lower limit to define the region of interest, the cropbox
@maxPoint -> upper limit to define the region of interest, the cropbox

Returns:
@cloud_wihtout_rooftop -> cloud after applying a voxel grid, selection cropbox (region of interest) and deleting the rooftop points.

Function: This function receives a cloud and applies voxel grid to reduce the density of the points (downsample the data) and then applies cropbox to only select a region of interest (meaning that instead of having points of the area of 300 meters surrounding the lidar, now just around 50 meters). It also removes the points that collide with rooftop
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //* DOWNSAMPLE THE DATA (VOXEL GRID)
    // Create the filtering object 
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    //* CROPBOX FOR SELECTING REGION OF INTEREST
    // cropbox receives dimensions and returns the cloud points that reside inside that dimensions
    typename pcl::PointCloud<PointT>::Ptr cloud_cropbox (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> cropBoxFilter (true);
    cropBoxFilter.setInputCloud (cloud_filtered);

    // Cropbox slighlty bigger then bounding box of points
    cropBoxFilter.setMin (minPoint);
    cropBoxFilter.setMax (maxPoint);

    // Cloud after cropbox
    cropBoxFilter.filter (*cloud_cropbox);

    //* REMOVE ROOF POINTS
    typename pcl::PointCloud<PointT>::Ptr cloud_without_roofPoints = RemoveRoofPoints(cloud_cropbox);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    if (OUTPUT_LOGS)
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_without_roofPoints;

}


/*
Params:
@inliers -> Indices of the points that are part of the plane
@cloud -> The complete cloud that includes the objects and the road

Return:
segResults -> pair of clouds: 1. Cloud ptr to the objects 2. Cloud ptr to the road

Function: What this code does is separate point clouds. The plane would be the road, and the inliers are the points that form part of the plane/road. So by extracing the points taht are not inliers (not part of road), you obtain the obstacles. 
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::ExtractIndices<PointT> extract; // create object that make extractions
    extract.setInputCloud (cloud);    

    typename pcl::PointCloud<PointT>::Ptr plane_points {new pcl::PointCloud<PointT> ()};
    extract.setIndices (inliers);
    extract.filter (*plane_points); // Filter the plane points and save them

    typename pcl::PointCloud<PointT>::Ptr objects_points {new pcl::PointCloud<PointT>()};
    extract.setNegative (true); // Apply the same filter as before, but now excluding the inliers
    extract.filter(*objects_points);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(objects_points, plane_points);
    return segResult;
}

/* 
Params
@cloud : cloud to be segmented
@max iterations: number of iterations
@ditanceThreshold: determines how close a point must be to the model to be considered an inlier

Return
pair: 1. The clouds with all the point clouds that are part of the plane (road) the inliers 2. Return the complete cloud that was analyzed without segmentation 

Function: The separte clouds returns a pair with the inliers (the points that are part of the plane) and the cloud that was analyzed (so the full point cloud without segmentation)
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the segmentation object
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // ptr to the inliers
    
    //* CODE USING PCL BUILTIN RANSAC
    pcl::SACSegmentation<PointT> seg; 
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients}; // coefficients define what the plane is
    
    seg.setOptimizeCoefficients(true); // if true, then try to get the best model
    seg.setModelType (pcl::SACMODEL_PLANE); // telling you are looking for a plane model
    seg.setMethodType (pcl::SAC_RANSAC); // telling that the way of looking is ransac - random sample concensus. RANSAC is the importnat algorithm that runs behind everything
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    
    // Set the cloud and do the segmentation
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients); 
    

    //* CODE USING MY OWN RANSAC ALGORITHM
    // Segment<PointT> seg; 
    // seg.Ransac(inliers, cloud, maxIterations, distanceThreshold); // slower than built-in function

    // manage possible errors
    if (inliers->indices.size () == 0) // didnt found any model that fits the data
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    if (OUTPUT_LOGS)
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // The separte clouds returns a pair with the inliers (the points that are part of the plane) and the cloud that was analyzed (so the full point cloud without segmentation)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

/*
Params:
cloud -> the original cloud that contains all the objects without separation
clusterTolerance -> Refers to the minimum distance between points to be consider part of same cluster. value to low will create many clusters from one object, value to high multiple objects as one cluster.
minsize, maxSize -> minimum and maximum amount of points for something to be a cluster.

Returns:
Returns a vector of cloud points where each cloud refers to a single object of the original point cloud

Function:
This function is in charge of traversing the cloud of objects and separating each object. 
*/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float cluster_tolerance, int min_size, int max_size)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointIndices> clusters_indices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //* PCL TREE AND CLUSTERING IMPLEMENTATION
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    // Extracting all the different clusters and saving their indices of each cluster. PointIndices is a vector of indices
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance); // value to low will create many clusters from one object, value to high multiple objects as one cluster.
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree); // Search clusters using kdtree
    ec.setInputCloud(cloud);
    ec.extract(clusters_indices);

    //* MY OWN IMPLEMENTATION OF TREE AND CLUSTERING ALGORITHMS
    // EuclideanCluster<PointT> clustering;
    // clustering.setInputCloud(cloud, cluster_tolerance);
    // clustering.euclideanCluster(clusters_indices);

    // For each cluster, iterate through the indices and get the original points of the cloud
    for (const auto& cluster : clusters_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster { new pcl::PointCloud<PointT> };

        // Iterate each index of cluster and save it in the cluster
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back(cloud->points[idx]);
        }

        // set meta information
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    if (OUTPUT_LOGS)
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}