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

	void traverse(Node* &node, std::vector<float> target, float distanceTol, std::vector<int> &ids, int level) {
		if (node == NULL) // return when is null, terminating decision of recursion
			return;
		
		float target_x = target[0];
		float target_y = target[1];

		// define the boundaries of the box around the target
		float minX = target_x - distanceTol;
		float maxX = target_x + distanceTol;
		float minY = target_y - distanceTol;
		float maxY = target_y + distanceTol;

		// the current value of the node
		int curr_id = node->id;
		float curr_x = node->point[0];
		float curr_y = node->point[1];

		// bool variables to know if current node is within boundaries
		bool currx_within = minX <= curr_x && curr_x <= maxX;
		bool curry_within = minY <= curr_y && curr_y <= maxY;
		bool within_box = currx_within && curry_within;

		// go left if max value is less than curr axes value. go right when min value is greater than curr axes value. 
		int indx = level % 2;
		bool go_left = (target[indx] + distanceTol) < node->point[indx];
		bool go_right = (target[indx] - distanceTol) > node->point[indx];
		
		level += 1; // increase level for next call
		
		// Just need to check if is within box, to add it. If not, just omit and continue with code
		if (within_box) {
			float distance = sqrt( (target_x - curr_x)*(target_x - curr_x) + (target_y - curr_y)*(target_y - curr_y));
			
			// check if distance is within distance tolerance
			if (distance <= distanceTol)
				ids.push_back(curr_id);
		}
		
		// Both of the conditionals are independent. Can occur that both get executed (when within boundaries) or just one of them

		if (go_left || within_box) { // if only need to go left
			traverse(node->left, target, distanceTol, ids, level);
		} 

		if (go_right || within_box) { // if only need to go and check right boundary
			traverse(node->right, target, distanceTol, ids, level);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		traverse(root, target, distanceTol, ids, 0);
		return ids;
	}
	

};




