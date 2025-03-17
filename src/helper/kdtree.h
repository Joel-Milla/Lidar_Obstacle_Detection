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
		void traverse(Node<PointT>* &node, PointT target, float distance_tol, std::vector<int> &ids, int level);
		void insert(PointT point, int id);
		Node<PointT>* root;
		void setTree(std::vector<Node<PointT>> cloud);

		//* To know if in xyz plane or only xy
		static constexpr int dimensions = traits::getDimensions<PointT>();

		//* Helper functions
		bool firstPointGreater(PointT p1, PointT p2, int depth);
		bool firstPointWithinRangeSecond(const Node<PointT>& point, const Node<PointT>& reference_point, int distanceTol) const;
		bool withinDistance(Node<PointT> point, Node<PointT> reference_point, int distanceTol);

	public:

		KdTree();
		~KdTree();
		
		void setTree(typename pcl::PointCloud<PointT>::Ptr cloud);
		std::vector<int> search(PointT target, float distance_tol);
	};
}

#endif
