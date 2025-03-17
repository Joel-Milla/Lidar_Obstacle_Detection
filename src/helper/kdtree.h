/* \author Joel Milla */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_

#include <cmath>
#include <vector>
#include <stdlib.h>
#include <pcl/point_cloud.h>
// to check traits of PointT below
#include "type_traits.h" 

namespace KdTreeSpace {
	// Structure to represent node of kd tree
	template<typename PointT>
	class Node {
	public:
		PointT point;
		int indx;
		Node* left;
		Node* right;

		Node(); // Another constructor so can set an empty node when initializing vectors and not throw an error -> std::vector<Node<PointT>> nodes();
		Node(PointT point, int indx);
	};

	template <typename PointT>
	class KdTree {
	private:
		//* Main functions used in public methods
		void traverse(Node<PointT>* &node, PointT target, std::vector<int> &ids, int level);
		void insert(PointT point, int id);
		void setTree(const std::vector<Node<PointT>>& cloud);
		
		//* To know if in xyz plane or only xy
		Node<PointT>* root;
		static constexpr int dimensions = traits::getDimensions<PointT>();
		float distance_tol;

		//* Helper functions
		bool firstPointGreater(PointT p1, PointT p2, int depth);
		bool firstPointWithinRangeSecond(const PointT& point, const PointT& reference_point) const;
		bool withinDistance(PointT point, PointT reference_point);

	public:

		KdTree();
		~KdTree();
		
		void setTree(typename pcl::PointCloud<PointT>::Ptr cloud);
		std::vector<int> search(PointT target);
		void setDistanceTol(float distance_tol);
	};
}

#endif
