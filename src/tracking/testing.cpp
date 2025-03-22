#include <filesystem>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h> // for transformPointCloud
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <boost/format.hpp>

#include <mutex>
#include <thread>

#include "../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../processPointClouds.cpp"
#include "../render/render.h"

// Needed for tracking
#include <pcl/tracking/kld_adaptive_particle_filter.h>


using namespace pcl::tracking;
using namespace std::chrono_literals;

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud;

std::mutex mtx_;
ParticleFilter::Ptr tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;


//Filter along a specified dimension
void filterPassThrough(const CloudConstPtr &cloud, Cloud &result) {
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-5.0, 20.0);  // Wider range (-5m to 20m instead of 0-10m)
  pass.setKeepOrganized(false);
  pass.setInputCloud(cloud);
  pass.filter(result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void renderPointTracking(pcl::visualization::PCLVisualizer::Ptr& viewer, const CloudPtr& cloud, std::string name) {
  // Select color based off input value
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "input cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "input cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");

  //* Render particles
  // ParticleXYZRPY result = tracker_->getResult ();
  tracker_->setInputCloud (cloud);
  tracker_->compute ();

  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  std::cout << particles << std::endl;

  if (particles) {
    std::cout << "Got particles!" << std::endl;
    //Set pointCloud with particle's points
    pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for (const auto& particle: *particles) {
      pcl::PointXYZ point;
            
      point.x = particle.x;
      point.y = particle.y;
      point.z = particle.z;
      particle_cloud->push_back (point);
    }
    // Create a color handler
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(particle_cloud, 255, 0, 0);

    // Try update first, add only if needed
    if (!viewer->updatePointCloud(particle_cloud, red_color, "particles")) {
        viewer->addPointCloud(particle_cloud, red_color, "particles");
    }
    
    // Set size separately (this still makes sense as a separate call)
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "particles");

    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

    //move close to camera a little for better visualization
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
    CloudPtr result_cloud (new Cloud ());
    pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

    //Draw blue model reference point cloud
    {
      pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

      if (!viewer->updatePointCloud (result_cloud, blue_color, "resultcloud"))
        viewer->addPointCloud (result_cloud, blue_color, "resultcloud");
    }
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

int
main (int argc, char** argv) {
  //read pcd file
  ProcessPointClouds<pcl::PointXYZRGBA>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZRGBA>();
  CloudPtr target_cloud;
  
  std::string name_file = "/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/tracking/cyclist.pcd";
  target_cloud = pointProcessorI->loadPcd(name_file);

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  downsampling_grid_size_ =  0.002;

  std::vector<double> default_step_covariance = std::vector<double>(6, 0.05 * 0.05);  // Was 0.015 * 0.015
  default_step_covariance[3] *= 40.0;  // Roll
  default_step_covariance[4] *= 40.0;  // Pitch
  default_step_covariance[5] *= 40.0;  // Yaw
  

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

  ParticleT bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;


  /*
  First, in main function, these lines set the parameters for tracking.
  */
  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (1000);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (600);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal (false);


  /*
  Here, we set likelihood function which tracker use when calculate weights. You can add more likelihood function as you like. By default, there are normals likelihood and color likelihood functions. When you want to add other likelihood function, all you have to do is initialize new Coherence Class and add the Coherence instance to coherence variable with addPointCoherence function.
  */
  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence
    (new ApproxNearestPairPointCloudCoherence<RefPointType>);
  
  DistanceCoherence<RefPointType>::Ptr distance_coherence
    (new DistanceCoherence<RefPointType>);
  coherence->addPointCoherence (distance_coherence);

  pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (0.01));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance(0.05);

  tracker_->setCloudCoherence (coherence);

  /*
  In this part, we set the point cloud loaded from pcd file as reference model to tracker and also set model’s transform values.
  */
  //prepare the model of tracker's target
  Eigen::Vector4f c;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);

  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);

  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());

  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = FPS;
  initCamera(setAngle, viewer);

  //* STREAM OF PCD
  std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2"); // chronollogical order vector of all file names containing PCD. 
  auto streamIterator = stream.begin() + 50;
  CloudPtr inputCloudI;

  while (!viewer->wasStopped ())
  {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      //* Load pcd and run cloud preprocessing. We separate preprocessing and tracking. One is in charge of downsampling and separating objects from the plane
      std::string name_file = (*streamIterator).string();
      inputCloudI = pointProcessorI->loadPcd(name_file);

      //* Pair that contains point cloud of (cluster of objects, plane)
      // std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_plane = preProcessing(pointProcessorI, viewer, inputCloudI);

      // objectTracking(pointProcessorI, viewer, object_to_track, clusters_plane.first, clusters_plane.second);
      renderPointTracking(viewer, inputCloudI, name_file);
      // viewer->spin();
            
      streamIterator++;
      if(streamIterator == stream.end())
          streamIterator = stream.begin();

      viewer->getRenderWindow()->Render();
      // Add a small delay to control frames
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}