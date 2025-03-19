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
		~Node();
	};

	template <typename PointT>
	class KdTree {
	private:
		//* Main functions used in public methods
		void insert(PointT point, int indx);

		template <class Iterator>
		void setTree(Iterator first, Iterator last);
		void deleteTree(Node<PointT>* node);

		template<class Iterator>
		void setTree(Iterator first, Iterator last, int axis);
		
		//* To know if in xyz plane or only xy
		Node<PointT>* root;
		static constexpr int dimensions = traits::getDimensions<PointT>();
		float distance_tol;

		//* Helper functions
		bool firstPointGreater(PointT first_point, PointT second_point, int depth);
		bool firstPointWithinRangeSecond(const PointT& point, const PointT& reference_point) const;
		bool withinDistance(const PointT& point, const PointT& reference_point) const;

	public:

		KdTree();
		~KdTree();
		
		void setTree(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
		std::vector<int> search(const PointT& target) const;
		void setDistanceTol(float distance_tol);
	};
}


#endif
