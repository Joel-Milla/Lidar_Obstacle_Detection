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
    void proximity(int indx, std::vector<int>& cluster,  std::unordered_set<int>& pointsProcessed, KdTreeSpace::KdTree<PointT>* tree, float distanceTol, const std::vector<std::vector<float>>& points);
        
    KdTreeSpace::KdTree<PointT> tree;
public:
    EuclideanCluster();
    ~EuclideanCluster();
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud);

    pcl::PointIndices euclideanCluster(const std::vector<std::vector<float>>& points, KdTreeSpace::KdTree<PointT>* tree, float distanceTol);
};

#endif