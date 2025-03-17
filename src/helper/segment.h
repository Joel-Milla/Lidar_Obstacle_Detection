
// PCL lib Functions for processing point clouds 

#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "../processPointClouds.h"

template<typename PointT>
class Segment {
    public:
    Segment();
    ~Segment();
    
    // Methods
    void Ransac(pcl::PointIndices::Ptr inliers,  typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
    
};


#endif /* SEGMENT_H_ */