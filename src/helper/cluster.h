#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <unordered_set>
#include "kdtree.h"

namespace Cluster {

    template <typename PointT>
    void helperProximty(int indx, std::vector<int>& cluster, 
        std::unordered_set<int>& pointsProcessed, 
        KdTreeSpace::KdTree<PointT>* tree, float distanceTol, 
        const std::vector<std::vector<float>>& points);
        
    template <typename PointT>
    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTreeSpace::KdTree<PointT>* tree, float distanceTol);
}

#endif