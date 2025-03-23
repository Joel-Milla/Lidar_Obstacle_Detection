# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Project 
We first create a simulator of the Lidar throwing beams in the space, for future processing. 
<img src="./README_FILES/simulator.png" width="700" height="400"/>

Then, we modify the simulator to only render the point cloud data based on the rays that were previously created.
<img src="./README_FILES/simulator_point_cloud.png" width="700" height="400"/>

Then, we used segmentation methods using RANSAC algorihtm and extraction methods to separate the plane and the objects detected from the point cloud. AFter that, the results of the different objects detected around the lidar are shown in the simulator.

<img src="./README_FILES/segmentation.png" width="700" height="400"/>

Next, we implemented euclidan clustr exraction from PCL to obtain the different clusters of the PCD. Also, we dislayed each different cluster as shown in image belwo about different objects around lidar in simulator.

<img src="./README_FILES/clustering.png" width="700" height="400"/>

And now displaying also the plane/road that was segmented at first using plane model segmentation usig RANSAC, we obtain our objects and plane in the next shape:

<img src="./README_FILES/complete_cluster.png" width="700" height="400"/>

After that, I implemented my own Kd tree algorithm for insertion and searching. And use that implmementation and constructed my own euclidan clustering algorithm to separate between datapoints. This is a result with 2D data.

<img src="./README_FILES/euclidean_cluster_manual.png" width="700" height="400"/>

For the last part, I implemented boudning boxes around the objects that exists in the plane. 

<img src="./README_FILES/objects.png" width="700" height="400"/>

After all of this, we now render real PCD data to work and apply the functions that were previously created. An initial view of the PCD can be seen below of a city block.

<img src="./README_FILES/real_pcd.png" width="700" height="400"/>

For extra cleaning and easier processing, we also created a function to remove the points that the lidar was reading from the roof of the car. We also applied vox grid filtering to dowsample the points of our PCD for easier calculations and selected a region of interest to only focus on the impornat parts of the point cloud. After all this, we also used function to filter and extract the point cloud information of the roof. 

<img src="./README_FILES/after_cleaning.png" width="700" height="400"/>

The next steps where preparing image for obstacle detections: 

1. Segment the filter cloud in two parts, the road and the obstacles. We used the previous segmentined algorhtms to segement the objects from the road. Furtheremo, we display the roof of the car in a box to show the region of the roof of the car that was removed from clusters. 

<img src="./README_FILES/step1.png" width="700" height="400"/>

2. Step 2 of obstacle detection is clustiner the obstacle cloud. We cluster the objects based on the proximity of the neighbors using the previous fucniton created using euclidean clustering optimized by kd tree. Diferent thresholds and min/max values were used to find and fit the best separation between objects. 

<img src="./README_FILES/step2.png" width="700" height="400"/>

3. Step 3 was applying bounding boxes around individual clusters. 

<img src="./README_FILES/step3.png" width="700" height="400"/>

4. The final step we created a stream of multiple pcd data to streamline it as a streaming video and apply in each frame the detection algorithm for multiple objects.

<img src="./README_FILES/detection.gif" width="700" height="400" />
<img src="./README_FILES/video2.gif" width="700" height="400" />

5. After all the previous steps, I implemented a final functionality using a different pcd files to track a cyclist through consecutive frames. For this tracking, I utilized PCL's Particle Filter Algorithm The red dots visible in the visualization represent the particles showing potential positions, while the blue points show the transformed reference model at the estimated position. Below can be seen the result of the cyclist being tracked through the sequence.

<img src="./README_FILES/tracking.gif" width="700" height="400" />

### Sensor Fusion course for self-driving cars.

This course is about sensor fusion, which is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. The focus is on two sensors, lidar, and radar.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resolution imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

## Workspace

The workspace provided projects. Versions used are as follows:

* Ubuntu 22.04
* PCL - v1.12
* C++ v17


## Local Installation

### Ubuntu 

1. Clone this github repo:

   ```sh
   cd ~
   git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
   ```

2.  Edit [CMakeLists.txt](https://github.com/udacity/SFND_Lidar_Obstacle_Detection/blob/master/CMakeLists.txt) as follows:

   ```cmake
cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

project(pcl)

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/quiz/cluster/cluster.cpp src/quiz/cluster/kdtree.cpp src/helper/kdtree.cpp) 
target_link_libraries (environment ${PCL_LIBRARIES})
   ```

3. Execute the following commands in a terminal

   ```shell
   sudo apt install libpcl-dev
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake ..
   make
   ./environment
   ```

   This should install the latest version of PCL. You should be able to do all the project with this setup.

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_macosx.html#compiling-pcl-macosx)
