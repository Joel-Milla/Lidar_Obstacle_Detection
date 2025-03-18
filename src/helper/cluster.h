#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include "kdtree.h"
#include "kdtree.cpp"
#include <pcl/PointIndices.h>
#include <boost/dynamic_bitset.hpp>

template <typename PointT>
class EuclideanCluster {
private:
    void proximity(pcl::PointIndices &cluster, boost::dynamic_bitset<>& points_processed, int point_indx) const;
        
    KdTreeSpace::KdTree<PointT> tree;
    typename pcl::PointCloud<PointT>::Ptr input_cloud;
public:
    EuclideanCluster();
    ~EuclideanCluster();
    void setInputCloud(typename pcl::PointCloud<PointT>::Ptr input_cloud, float distance_tol);

    void euclideanCluster(std::vector<pcl::PointIndices> &cluster_indices) const;
};


#endif