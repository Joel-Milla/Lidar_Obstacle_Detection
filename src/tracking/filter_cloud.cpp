#include "../render/render.h"
#include "../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../processPointClouds.cpp"
#include <boost/concept/detail/has_constraints.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <thread>

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
    // Apply filter cloud to downsample the amount of data in PCD for faster data management by applying voxel grid, then use cropbox to reduce field of view just near the car, after that remove roof points.s
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-10, -10, -10, 1), Eigen::Vector4f ( 26, 10, 10, 1));

    //* STEP1: SEGMENT PLANE VS OBJECTS VS ROOF
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    
    //* STEP2: CLUSTER OBSTACLE CLOUD
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.55, 30, 600);

    return {cloudClusters, segmentCloud.second};
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

        if (indx == 1) {
            // saveCluster(cluster);
            renderBox(viewer, box, indx);
        }
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
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2"); // chronollogical order vector of all file names containing PCD. 
    auto streamIterator = stream.begin() + 50;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    pcl::PointCloud<pcl::PointXYZI>::Ptr object_to_track = pointProcessorI->loadPcd("/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/tracking/cyclist.pcd");

    // objectTracking(pointProcessorI, viewer, object_to_track, clusters_plane.first, clusters_plane.second);
    // viewer->spin();

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //* Load pcd and run cloud preprocessing. We separate preprocessing and tracking. One is in charge of downsampling and separating objects from the plane
        std::string name_file = (*streamIterator).string();
        inputCloudI = pointProcessorI->loadPcd(name_file);

        //* Pair that contains point cloud of (cluster of objects, plane)
        std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_plane = preProcessing(pointProcessorI, viewer, inputCloudI);

        objectTracking(pointProcessorI, viewer, object_to_track, clusters_plane.first, clusters_plane.second);
        // renderPointCloud(viewer, inputCloudI, name_file);
        viewer->spin();
            
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->getRenderWindow()->Render();
        // Add a small delay to control frames
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}