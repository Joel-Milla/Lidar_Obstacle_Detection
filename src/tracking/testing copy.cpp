#include <filesystem>
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

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

std::mutex mtx_;
ParticleFilter::Ptr tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;


//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}


//Draw the current particles
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  /*
  In drawParticles function, you can get particles’s positions by calling getParticles().
  */
    
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
  {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for (const auto& particle: *particles)
	{
	  pcl::PointXYZ point;
          
	  point.x = particle.x;
	  point.y = particle.y;
	  point.z = particle.z;
	  particle_cloud->push_back (point);
	}

      //Draw red particles 
      {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

	if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
	  viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}

//Draw model reference point cloud
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{
  /*
  In drawResult function, you can get model information about position and rotation.
  */
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}

//visualization's callback function
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  std::lock_guard<std::mutex> lock (mtx_);

  if (!cloud_pass_) {
      std::this_thread::sleep_for(1s);
      return;
  }

  //Draw downsampled point cloud from sensor    
  if (new_cloud_ && cloud_pass_downsampled_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_downsampled_;
    
      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
	{
	  viz.addPointCloud (cloud_pass, "cloudpass");
	  viz.resetCameraViewpoint ("cloudpass");
	}
      bool ret = drawParticles (viz);
      if (ret)
        drawResult (viz);
    }
  new_cloud_ = false;
}

//OpenNI Grabber's cloud Callback function
void
cloud_cb (const CloudConstPtr &cloud)
{
  std::lock_guard<std::mutex> lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  /* 
  Until the counter variable become equal to 10, we ignore the input point cloud, because the point cloud at first few frames often have noise. After counter variable reach to 10 frame, at each loop, we set downsampled input point cloud to tracker and the tracker will compute particles movement.
  */
  if(counter < 10){
	counter++;
  }else{
  	//Track the object
	tracker_->setInputCloud (cloud_pass_downsampled_);
	tracker_->compute ();
	new_cloud_ = true;
  }
}

// Add this function to convert PointXYZI cloud to PointXYZRGBA
CloudPtr convertPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
  CloudPtr output_cloud(new Cloud);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;
  output_cloud->is_dense = input_cloud->is_dense;
  output_cloud->points.resize(input_cloud->size());
  
  // Map intensity to a grayscale RGB value
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    output_cloud->points[i].x = input_cloud->points[i].x;
    output_cloud->points[i].y = input_cloud->points[i].y;
    output_cloud->points[i].z = input_cloud->points[i].z;
    
    // Convert intensity to color (grayscale)
    uint8_t gray = static_cast<uint8_t>(std::min(255.0f, input_cloud->points[i].intensity * 255.0f / 100.0f));
    output_cloud->points[i].r = gray;
    output_cloud->points[i].g = gray;
    output_cloud->points[i].b = gray;
    output_cloud->points[i].a = 255; // Fully opaque
  }
  
  return output_cloud;
}

int
main (int argc, char** argv)
{
  if (argc < 3)
    {
      PCL_WARN("Please set device_id pcd_filename(e.g. $ %s '#1' sample.pcd)\n", argv[0]);
      exit (1);
    }

  //read pcd file
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_intensity;
  target_cloud.reset(new Cloud());
  
  std::string name_file = "/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/tracking/cyclist.pcd";
  target_cloud_intensity = pointProcessorI->loadPcd(name_file);
  target_cloud = convertPointCloud(target_cloud_intensity);

  std::string device_id = std::string (argv[1]);  

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  downsampling_grid_size_ =  0.002;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

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
  coherence->setMaximumDistance (0.01);

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

  //Setup OpenNIGrabber and viewer
  pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
  // pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
  std::function<void (const CloudConstPtr&)> f = cloud_cb;
  // interface->registerCallback (f);

  std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("/home/jalej/Documents/Learning/courses/Lidar_Obstacle_Detection/src/sensors/data/pcd/data_2"); // chronollogical order vector of all file names containing PCD.
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  viewer_->runOnVisualizationThread (viz_cb, "viz_cb");

  //Start viewer and object tracking
  // interface->start();
  while (!viewer_->wasStopped ()) {

    std::string name_file = (*streamIterator).string();
    inputCloudI = pointProcessorI->loadPcd(name_file);
    CloudPtr convertedCloud = convertPointCloud(inputCloudI);

    cloud_cb(convertedCloud);

    // std::this_thread::sleep_for(1s);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();
  }
  // interface->stop();
}