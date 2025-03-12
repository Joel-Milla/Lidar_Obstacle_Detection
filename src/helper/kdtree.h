/* \author Joel Milla */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_
#include <cmath>
#include <vector>
#include <stdlib.h>

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

	class KdTree {
	// private:
	public:
		Node* root;

		KdTree();
		~KdTree();
		void traverse(Node* &node, std::vector<float> target, float distanceTol, std::vector<int> &ids, int level);
		void insert(Node *&node, std::vector<float> point, int id, int level);

		void insert(std::vector<float> point, int id);
		
		std::vector<int> search(std::vector<float> target, float distanceTol);
	};
}

#endif
