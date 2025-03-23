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


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/helper/cluster.cpp src/helper/kdtree.cpp src/helper/segment.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

# Following are commented out but can be used to generate executable that track cyclist
# add_executable (tracking src/tracking.cpp src/render/render.cpp src/processPointClouds.cpp src/helper/cluster.cpp src/helper/kdtree.cpp src/helper/segment.cpp)
# target_link_libraries (tracking ${PCL_LIBRARIES})
