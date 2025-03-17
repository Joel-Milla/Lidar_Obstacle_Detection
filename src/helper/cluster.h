#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <unordered_set>
// #include "kdtree.h"
#include "kdtree.cpp"
#include <pcl/PointIndices.h>

template <typename PointT>
class EuclideanCluster {
private:
    void proximity(pcl::PointIndices &cluster, std::unordered_set<int>& points_processed, int point_indx);
        
    KdTreeSpace::KdTree<PointT> tree;
    typename pcl::PointCloud<PointT>::Ptr input_cloud;
public:
    EuclideanCluster();
    ~EuclideanCluster();
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud, float distance_tol);

    void euclideanCluster(std::vector<pcl::PointIndices> &cluster_indices);
};

#endif