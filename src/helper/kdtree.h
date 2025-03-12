/* \author Joel Milla */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_

#include <cmath>
#include <vector>
#include <stdlib.h>
#include <pcl/point_cloud.h>

namespace KdTreeSpace {
	// Structure to represent node of kd tree
	class Node {
	public:
		std::vector<float> point;
		int id;
		Node* left;
		Node* right;

		Node(std::vector<float> arr, int setId);
		~Node();
	};

	template <typename PointT>
	class KdTree {
	private:
		void traverse(Node* &node, PointT target, float distanceTol, std::vector<int> &ids, int level);
		void insert(Node *&node, PointT point, int id, int level);
		void insert(PointT point, int id);

	public:
		Node* root;

		KdTree();
		~KdTree();
		
		void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
		PointT search(PointT, float distanceTol);
	};
}

#endif
