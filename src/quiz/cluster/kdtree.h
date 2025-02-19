/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(Node *&node, std::vector<float> point, int id, int level) {
		float x = point[0];
		float y = point[1];
		bool compare_x = level % 2 == 0;

		if (node == NULL) {
			Node* newNode = new Node(point, id);
			node = newNode;
			return; 
		}
		
		std::vector<float> curr_value = node->point;
		bool go_left = false;

		if (compare_x)
			go_left = x < curr_value[0] ? true : false;
		else
			go_left = y < curr_value[1] ? true : false;
		
		level += 1;
		if (go_left) {
			insert(node->left, point, id, level);
		}
		else {
			insert(node->right, point, id, level);
		}
   
		return;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insert(root, point, id, 0);

		

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




