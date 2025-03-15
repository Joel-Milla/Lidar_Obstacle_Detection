cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


# add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/quiz/cluster/cluster.cpp src/quiz/cluster/kdtree.cpp src/helper/kdtree.cpp) # Remove the cpps that are inside quiz
add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp src/helper/cluster.cpp src/helper/kdtree.cpp src/helper/segment.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})





