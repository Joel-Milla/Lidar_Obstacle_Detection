/* \author Joel Milla */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/*
Function:
Receives a viewer which is where the graphs will be displayed. This function only shows a very simple model. 
*/
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; // A value of true shows the cars in the scene, with false it removes the cars and street from simulator
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    //* Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan(); // This already returns a pointer, a smart pointer that manages memory deallocation, etc.

    // Render the point cloud
    Color color = Color(0.0,255.0,0.0);
    renderPointCloud(viewer, inputCloud, "Point Cloud", color);

    //* Point processor that contains all the methods for filtering, segmentaion, clustering, etc. 
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    
    //* Clustering process that clusters the different objects within the PCD
    // Pass to function Clustering all the points that are not part of the road/plane of the car
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1, 20, 500);
	
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        
        // pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

/**
 * @brief Function in charge of rendering the result of preprocessing for object detection
 * 
 * @param pointProcessorI object that has all the methods for processing the point cloud (like segmenting, clustering, etc)
 * @param viewer The viewer which is in charge of rendering the points
 * @param inputCloud The input cloud which the preprocessing will be applied to
 */
void cityBlock(ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // Obtain roof points' cloud, and its indices
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointIndices::Ptr> roof = pointProcessorI->ObtainRoofPoints(inputCloud);

    // Apply filter cloud to downsample the amount of data in PCD for faster data management by applying voxel grid, then use cropbox to reduce field of view just near the car, after that remove roof points.s
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-10, -10, -10, 1), Eigen::Vector4f ( 26, 10, 10, 1));
    
    //* STEP1: SEGMENT PLANE VS OBJECTS VS ROOF
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);


    //* STEP2: CLUSTER OBSTACLE CLOUD
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.55, 30, 600);

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

        renderBox(viewer, box, indx);
        // renderBox(viewer, boxQ, indx);
    }

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
    if (OUTPUT_LOGS)
        std::cout << "Starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //* STREAM OF PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1"); // chronollogical order vector of all file names containing PCD. 
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
