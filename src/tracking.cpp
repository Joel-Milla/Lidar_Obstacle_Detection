/* \author Joel Milla */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
// #include "render/render.cpp"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <cstddef>
#include <pcl/impl/point_types.hpp>
#include <string>
#include <thread>

// Tracking libraries
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter.h>

typedef pcl::PointXYZRGBA RefPointType;
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::tracking::ParticleFilterTracker<RefPointType, ParticleT>
    ParticleFilter;

std::vector<ParticleFilter::Ptr> tracking_objects;

std::size_t indx_to_show = 4;

/**
 * @brief Create a Tracker object which will be stable during all frames
 * 
 * @param target_cloud that will be checked throughout the frames 
 * @return ParticleFilter::Ptr pointer to the tracker
 */
ParticleFilter::Ptr createTracker(const CloudPtr &target_cloud) {
    //* Noise model for particle algorithm
    std::vector<double> default_step_covariance =
        std::vector<double>(6, 0.05 * 0.05); // Was 0.015 * 0.015
    default_step_covariance[3] *= 40.0;      // Roll
    default_step_covariance[4] *= 40.0;      // Pitch
    default_step_covariance[5] *= 40.0;      // Yaw

    std::vector<double> initial_noise_covariance =
        std::vector<double>(6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

    //* Adjusts the number of particles needed based on the complexity of the distribution
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType,
                                                        ParticleT>::Ptr
        tracker(
            new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<RefPointType,
                                                                    ParticleT>(8));
    ParticleFilter::Ptr tracker_;

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;

    // Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum(1000);
    tracker->setDelta(0.99);
    tracker->setEpsilon(0.2);
    tracker->setBinSize(bin_size);

    // Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(1);
    tracker_->setParticleNum(600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal(false);

    /*
    Here, we set likelihood function which tracker use when calculate weights. You
    can add more likelihood function as you like. By default, there are normals
    likelihood and color likelihood functions. When you want to add other
    likelihood function, all you have to do is initialize new Coherence Class and
    add the Coherence instance to coherence variable with addPointCoherence
    function.
    */
    //* How PCL calculates particle weights
    pcl::tracking::ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
        coherence(new pcl::tracking::ApproxNearestPairPointCloudCoherence<
                    RefPointType>);

    pcl::tracking::DistanceCoherence<RefPointType>::Ptr distance_coherence(
        new pcl::tracking::DistanceCoherence<RefPointType>);
    coherence->addPointCoherence(distance_coherence);

    pcl::search::Octree<RefPointType>::Ptr search(
        new pcl::search::Octree<RefPointType>(0.01));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(0.05);

    tracker_->setCloudCoherence(coherence);

    /*
    In this part, we set the point cloud loaded from pcd file as reference model
    to tracker and also set model’s transform values.
    */
    //* Compute centroid of object, because tracking algorithm will track object relative to its centroid
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();

    CloudPtr transed_ref(new Cloud);
    CloudPtr transed_ref_downsampled(new Cloud);

    pcl::compute3DCentroid<RefPointType>(*target_cloud, c);

    trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

    //* Transforms cloud to be centered to the origin, to make calculations simpler
    pcl::transformPointCloud<RefPointType>(*target_cloud, *transed_ref,
                                            trans.inverse());
    // set reference model and trans
    tracker_->setReferenceCloud(transed_ref);
    tracker_->setTrans(trans);

    return tracker;
}

/**
 * @brief Function in charge of getting first frame, obtaining trackers, and saving them
 *
 * @param pointProcessorI object that has all the methods for processing the
 * point cloud (like segmenting, clustering, etc)
 * @param viewer The viewer which is in charge of rendering the points
 * @param inputCloud The input cloud which the preprocessing will be applied to
 * @param firstFrame Boolean value that indicates if this is the first frame, to
 * set the objects to be tracked
 */
void processFirstFrame(ProcessPointClouds<pcl::PointXYZRGBA> *pointProcessorI,
                       pcl::visualization::PCLVisualizer::Ptr &viewer,
                       CloudPtr &cloudObjects) {
    tracking_objects.clear();
    //* CLUSTER OBSTACLE CLOUD
    std::vector<CloudPtr> cloudClusters =
        pointProcessorI->Clustering(cloudObjects, 0.55, 30, 600);
    tracking_objects.reserve(cloudClusters.size());

    //* RENDER BOUNDING BOXES AROUND OBJECTS
    for (std::size_t indx = 0; indx < cloudClusters.size(); indx++) {
        if (indx != indx_to_show) continue;

        CloudPtr &cluster = cloudClusters[indx];
        ParticleFilter::Ptr tracker = createTracker(cluster);
        tracking_objects.push_back(tracker);
    }
}

/**
 * @brief Recieves a cloud and its parameters, and is in charge of rendering it to the viewer
 * 
 * @tparam PointT 
 * @param viewer 
 * @param cloud cloud to be rendered
 * @param name id of the cloud in the view
 * @param color 
 * @param size size of the point in the viewer
 */
template <typename PointT>
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string name, Color color, double size) {
	// viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, name);
	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	
	// Create a color handler
	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(cloud, color.r*255, color.g*255, color.b*255);
	// Try update first, add only if needed
	if (!viewer->updatePointCloud(cloud, colorHandler, name)) {
		viewer->addPointCloud(cloud, colorHandler, name);
	}
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
}

/**
 * @brief Function in charge of tracking objects across different views. If found the object in the new frame, then it keeps tabs on it
 * 
 * @param viewer The viewer that will display the point cloud
 * @param tracker_ The one in charge of tracking the object through the cloud
 * @param cloud Cloud of obstacles where the tracking will happen
 * @param id Id to unique identify the object being tracked
 */
void drawTrackedObject(pcl::visualization::PCLVisualizer::Ptr &viewer, const ParticleFilter::Ptr tracker_, const CloudPtr& cloud, size_t id) {
    //* Render particles
    tracker_->setInputCloud (cloud);
    tracker_->compute ();

    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();

    //* Makes best estimate for objects position
    if (particles) {
        //Set pointCloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (const auto& particle: *particles) {
            pcl::PointXYZ point;
                
            point.x = particle.x;
            point.y = particle.y;
            point.z = particle.z;
            particle_cloud->push_back (point);
        }

        renderPointCloud<pcl::PointXYZ>(viewer, particle_cloud, ("particles" + std::to_string(id)), Color(1,0,0), 3);

        ParticleT result = tracker_->getResult ();
        Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

        //move close to camera a little for better visualization
        transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
        CloudPtr result_cloud (new Cloud ());
        pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

        renderPointCloud<RefPointType>(viewer, result_cloud, ("reference" + std::to_string(id)), Color(0,0,1), 3);
    }
}

/**
 * @brief Receives the cloud of objects and plane, and is in charge of calling the trackers to track and calling the render functions 
 * 
 * @param pointProcessorI Object contain function to process the point cloud
 * @param viewer 
 * @param cloudObjects 
 * @param cloudPlane 
 */
void renderCloud(ProcessPointClouds<pcl::PointXYZRGBA> *pointProcessorI,
                       pcl::visualization::PCLVisualizer::Ptr &viewer,
                       CloudPtr &cloudObjects, CloudPtr &cloudPlane) {

    for (std::size_t indx = 0; indx < tracking_objects.size(); indx++) {
        ParticleFilter::Ptr tracker_ = tracking_objects[indx];
        drawTrackedObject(viewer, tracker_, cloudObjects, indx);
    }

    //* Render both objects and plane
    renderPointCloud<pcl::PointXYZRGBA>(viewer, cloudObjects, "cloud_objects", Color(1,1,1), 4);
    renderPointCloud<pcl::PointXYZRGBA>(viewer, cloudPlane, "cloud_plane", Color(0,1,0), 4);

}

/**
 * @brief Sets the angle of the position and angle of the camera based on the
 * enums/value received
 *
 * @param setAngle enum of XY, TopDown, Side, FPS
 * @param viewer The view
 */
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-7, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
    //* Number four is the cyclist
    // indx_to_show = std::stoi(argv[1]);

    if (OUTPUT_LOGS)
        std::cout << "Starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //* STREAM OF PCD
    ProcessPointClouds<pcl::PointXYZRGBA> *pointProcessorI =
        new ProcessPointClouds<pcl::PointXYZRGBA>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd(
        "../src/sensors/data/pcd/data_2"); // chronollogical order vector of all
                                            // file names containing PCD.
    auto streamIterator = stream.begin();
    CloudPtr inputCloud;

    while (!viewer->wasStopped()) {
        //* Clear viewer for the new frame
        // viewer->removeAllPointClouds();
        // viewer->removeAllShapes();

        //* Load PCD file
        std::string name_file = (*streamIterator).string();
        inputCloud = pointProcessorI->loadPcd(name_file);

        //* APPLY DOWNSAMPLING + CROPBOX
        CloudPtr filterCloud = pointProcessorI->FilterCloud(
            inputCloud, 0.08f, Eigen::Vector4f(-10, -10, -10, 1),
            Eigen::Vector4f(26, 10, 10, 1));

        //* SEGMENT PLANE: OBJECTS VS ROOF
        std::pair<CloudPtr, CloudPtr> segmentCloud =
            pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

        //* If first frame, we need to create trackers
        if (streamIterator == stream.begin())
            processFirstFrame(pointProcessorI, viewer, segmentCloud.first);
        
        renderCloud(pointProcessorI, viewer, segmentCloud.first, segmentCloud.second);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        
        /*
        So spin is like an infinite loop. when it starts, it will freeze in that
        line of code, and run infinite loop that is in charge of showing the
        viewer and begin able to resize viewer screen, etc. and stops when you
        close the window, like hitting 'q'. The spinOnce is like spin but just one
        iteration, freezed the code and then continues with the code. can set the
        time
        */
        // viewer->spin();

        viewer->getRenderWindow()->Render();
        // Add a small delay to control frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
