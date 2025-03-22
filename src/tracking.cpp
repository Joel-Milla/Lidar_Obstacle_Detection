/* \author Joel Milla */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <string>
#include <thread>

// Tracking libraries
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter.h>



typedef pcl::PointXYZRGBA RefPointType;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

long long int indx_to_show = 0;

/**
 * @brief Function in charge of rendering the result of preprocessing for object detection
 * 
 * @param pointProcessorI object that has all the methods for processing the point cloud (like segmenting, clustering, etc)
 * @param viewer The viewer which is in charge of rendering the points
 * @param inputCloud The input cloud which the preprocessing will be applied to
 */
void cityBlock(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // Obtain roof points' cloud, and its indices
    // std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointIndices::Ptr> roof = pointProcessorI->ObtainRoofPoints(inputCloud);

    // Apply filter cloud to downsample the amount of data in PCD for faster data management by applying voxel grid, then use cropbox to reduce field of view just near the car, after that remove roof points.s
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.08f , Eigen::Vector4f (-10, -10, -10, 1), Eigen::Vector4f ( 26, 10, 10, 1));
    
    //* STEP1: SEGMENT PLANE VS OBJECTS VS ROOF
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    //* STEP2: CLUSTER OBSTACLE CLOUD
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.55, 30, 600);
    
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,1,1)};
    
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0)); // render plane cloud
    
    //* STEP3. RENDER BOUNDING BOXES AROUND OBJECTS
    for (std::size_t indx = 0; indx < cloudClusters.size(); indx++)
    {

        if (indx != indx_to_show) continue;
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster = cloudClusters[indx];
    
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(indx),colors[indx % 4]);
    
        // Apply bounding boxes surrounding the objects
        Box box = pointProcessorI->BoundingBox(cluster);
        // BoxQ boxQ = pointProcessorI->BoundingBoxQ(cluster);
    
        renderBox(viewer, box, indx);
        // renderBox(viewer, boxQ, indx);
    }
        
    renderPointCloud(viewer, segmentCloud.first, "obstCloud");
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud");
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


int main (int argc, char** argv)
{
    //* Number four is the cyclist
    indx_to_show = std::stoi(argv[1]);
    std::cout << indx_to_show << std::endl;
    if (OUTPUT_LOGS)
        std::cout << "Starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //* STREAM OF PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2"); // chronollogical order vector of all file names containing PCD. 
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // cityBlock(pointProcessorI, viewer, inputCloudI);

    while (!viewer->wasStopped())
    {
        
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detection process
        std::string name_file = (*streamIterator).string();
        inputCloudI = pointProcessorI->loadPcd(name_file);
        cityBlock(pointProcessorI, viewer, inputCloudI);
        // renderPointCloud(viewer, inputCloudI, name_file);
        
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spin();
    
        /*
        So spin is like an infinite loop. when it starts, it will freeze in that line of code, and run infinite loop that is in charge of showing the viewer and begin able to resize viewer screen, etc. and stops when you close the window, like hitting 'q'. 
        The spinOnce is like spin but just one iteration, freezed the code and then continues with the code. can set the time
        */
        // viewer->spinOnce(100, true);
        
        viewer->getRenderWindow()->Render();
        // Add a small delay to control frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
