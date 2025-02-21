/* \author Joel Milla */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_
#include <cmath>
#include <vector>
#include<stdlib.h>

namespace KdTree {
	// Structure to represent node of kd tree
	struct Node
	{
		std::vector<float> point;
		int id;
		Node* left;
		Node* right;

		Node(std::vector<float> arr, int setId);
		
		~Node();
	};

	class KdTree
	{
		Node* root;
		
		KdTree();
		~KdTree();

		public:
		void insert(Node *&node, std::vector<float> point, int id, int level);
		
		void insert(std::vector<float> point, int id);
		
		void traverse(Node* &node, std::vector<float> target, float distanceTol, std::vector<int> &ids, int level);
		
		std::vector<int> search(std::vector<float> target, float distanceTol);
	};
}

#endif
