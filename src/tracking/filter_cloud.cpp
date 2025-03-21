#include "../render/render.h"
#include "../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../processPointClouds.cpp"
#include <boost/concept/detail/has_constraints.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void saveCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
      pcl::io::savePCDFileASCII("../cyclist.pcd", *cluster);
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-7, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

/**
 * @brief Function receives a cloud, makes preprocessing (cleans the cloud) and returns a vector with the objects in the cloud, and the plane
 * 
 * @param pointProcessorI Object that has functionality to process cloud
 * @param viewer The one in charge of displaying the information
 * @param inputCloud Actual cloud
 * @return std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> -> 1. vector of objects, 2. Plane
 */
std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> preProcessing(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // Obtain roof points' cloud, and its indices
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointIndices::Ptr> roof = pointProcessorI->ObtainRoofPoints(inputCloud);

    // Apply filter cloud to downsample the amount of data in PCD for faster data management by applying voxel grid, then use cropbox to reduce field of view just near the car, after that remove roof points.s
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-10, -10, -10, 1), Eigen::Vector4f ( 26, 10, 10, 1));

    //* STEP1: SEGMENT PLANE VS OBJECTS VS ROOF
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    
    //* STEP2: CLUSTER OBSTACLE CLOUD
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.55, 30, 600);

    return {cloudClusters, segmentCloud.second};

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,1,1)};
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0)); // render plane cloud
    
    //* STEP3. RENDER BOUNDING BOXES AROUND OBJECTS
    for (std::size_t indx = 0; indx < cloudClusters.size(); indx++)
    {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster = cloudClusters[indx];

        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(indx),colors[indx % 4]);

        // Apply bounding boxes surrounding the objects
        Box box = pointProcessorI->BoundingBox(cluster);
        // BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);

        // if (indx == 5) saveCluster(cluster);
        renderBox(viewer, box, indx);
        // renderBox(viewer, boxQ, indx);
    }
}

void objectTracking(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr object_to_track,  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters, typename pcl::PointCloud<pcl::PointXYZI>::Ptr plane) {
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,1,1)};
    renderPointCloud(viewer, plane, "planeCloud", Color(0,1,0)); // render plane cloud
    
    //* STEP3. RENDER BOUNDING BOXES AROUND OBJECTS
    for (std::size_t indx = 0; indx < cloudClusters.size(); indx++)
    {
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster = cloudClusters[indx];

        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(indx),colors[indx % 4]);

        // Apply bounding boxes surrounding the objects
        Box box = pointProcessorI->BoundingBox(cluster);
        // BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);

        if (indx == 5)
            saveCluster(cluster);
        renderBox(viewer, box, indx);
        // renderBox(viewer, boxQ, indx);
    }
}

int main (int argc, char** argv)
{

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //* STREAM OF PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/app/src/sensors/data/pcd/data_2"); // chronollogical order vector of all file names containing PCD. 
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    std::string name_file = (*streamIterator).string();
    inputCloudI = pointProcessorI->loadPcd(name_file);
    std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_plane = preProcessing(pointProcessorI, viewer, inputCloudI);

    pcl::PointCloud<pcl::PointXYZI>::Ptr object_to_track = pointProcessorI->loadPcd("/app/src/tracking/cyclist.pcd");

    objectTracking(pointProcessorI, viewer, object_to_track, clusters_plane.first, clusters_plane.second);

    while (!viewer->wasStopped ())
    {

    // Clear viewer
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    // std::string name_file = (*streamIterator).string();
    // inputCloudI = pointProcessorI->loadPcd(name_file);
    // objectTracking(pointProcessorI, viewer, inputCloudI);
    // renderPointCloud(viewer, inputCloudI, name_file);
        
    // streamIterator++;
    // if(streamIterator == stream.end())
    //     streamIterator = stream.begin();

    viewer->spinOnce ();
    }
}