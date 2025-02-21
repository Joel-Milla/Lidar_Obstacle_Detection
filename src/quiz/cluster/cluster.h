#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include <unordered_set>
#include "kdtree.h"

namespace Cluster {

    void helperProximty(int indx, std::vector<int>& cluster, 
        std::unordered_set<int>& pointsProcessed, 
        KdTree::KdTree* tree, float distanceTol, 
        const std::vector<std::vector<float>>& points);
        
    std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree::KdTree* tree, float distanceTol);
}

#endif