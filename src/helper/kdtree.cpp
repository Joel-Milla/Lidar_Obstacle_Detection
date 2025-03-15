#include "kdtree.h"
#include "pcl/point_cloud.h"
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

// using namespace KdTreeSpace;
namespace KdTreeSpace {
    //* Node Implementation
    template <typename PointT>
    Node<PointT>::Node(PointT point, int indx) : point(point), indx(indx), left(nullptr), right(nullptr) {}

    template <typename PointT>
    Node<PointT>::Node(): point(), indx(0), left(nullptr), right(nullptr) {
    }

    //* KdTree Implementation
    template <typename PointT>
    KdTree<PointT>::KdTree() : root(NULL) {}

    template <typename PointT>
    KdTree<PointT>::~KdTree()
    {
        // delete root;
    }


    /*
    Params:
    @p1, @p2 => points to be compared with

    Returns:
    True/false depending if first point given is greater than the second one 

    Function:
    Receives two points, p1 and p2. Checks which depth of tree are, and then check based on the number of dimensions (xyz or xy) which variable neds to be compared with
    */
    template <typename PointT>
    bool KdTree<PointT>::firstPointGreater(PointT p1, PointT p2, int depth) {
        int axis_to_compare = depth % dimensions;

        switch (axis_to_compare) {
        
        case 0:
            return p1.x > p2.x;
        case 1:
            return p1.y > p2.y;
        case 2:
            return p1.z > p2.z;
        default:
            return false;
        }     
    }

    /*
    Input:
    @point => the point to be added
    @indx => indx of the point in the original cloud

    Return:
    void

    Function:
    Function where you receive the point you want to add. You will traverse the whole binary tree (go left if point is less than the current node, otherwise right) and set a leaf to the node receive.
    */
    template <typename PointT>
    void KdTree<PointT>::insert(PointT point, int indx) {

        //* For modifying the tree, we will use a pointer to pointer
        Node<PointT>** curr_node = &root;
        int level = 0;

        while (*curr_node != nullptr) {
            PointT curr_point = (*curr_node)->point;
            // std::cout << curr_point << "\n";
            // std::cout << point << "\n";
            bool first_point_greater = firstPointGreater(point, curr_point, level);

            if (first_point_greater)
                //* *curr_node -> returns the pointer to the node class. Because is a pointer, we need '->' to get right element. All that will return Node*, which is a pointer. By using '&' we are setting curr_node as the address of the pointer 
                curr_node = &((*curr_node)->right);
            else
                curr_node = &((*curr_node)->left);
            
            level++;
        }
        
        //* 'new' returns an address, and our curr_node that is a pointer to an address that is null, will change the address that is pointing at to this new pointer
        *curr_node = new Node<PointT>(point, indx);
    }

    /*
    Input:
    @cloud => preprocessed cloud data

    Return:
    void

    Function:
    This function receives the preprocess data which facillitates the construction of the tree
    */
    template <typename PointT> 
    void KdTree<PointT>::setTree(std::vector<Node<PointT>> cloud) {
        std::queue<std::pair<std::vector<Node<PointT>>, int> >* points_to_be_proccessed = new std::queue<std::pair<std::vector<Node<PointT>>, int> >();
        int axis = 0;
        points_to_be_proccessed->push({cloud, axis});

        //* This lambda function is a helper sort function to consider different axis (xyz)
        auto sort_by_axis = [&axis] (Node<PointT> p1, Node<PointT> p2) {
        switch (axis)
        {
        case 0:
            return p1.point.x > p2.point.x;
        case 1:
            return p1.point.y > p2.point.y;
        case 2:
            return p1.point.z > p2.point.z;
        default:
            return false;
        }
        };

        //* To avoid recursion and getting stack overflow, we will use a queue to sort iteratively the vector, get the middle point, insert, and continue. In a way that we fill each time every level
        while (!points_to_be_proccessed->empty()) {
            std::vector<Node<PointT>> points = points_to_be_proccessed->front().first;
            std::cout << points.size() << "\n";
            axis = points_to_be_proccessed->front().second;
            points_to_be_proccessed->pop();


            //* Sort points vector because we always want to pick the middle point
            std::sort(points.begin(), points.end(), sort_by_axis);
            
            //* Insert the middle point into the tree (to have a balanced tree)
            int size = points.size();
            int middle_indx = size / 2;

            PointT middle_point = points[middle_indx].point;
            int real_indx = points[middle_indx].indx;

            insert(middle_point, real_indx);

            //* Update queue so in next iteration insert the values that must be
            int new_axis = (axis + 1) % dimensions; // Must use modulo because in each vector we want to iterate between x, y, and z. This helps to keep count

            std::vector<Node<PointT>> left_vector;
            if (middle_indx > 0) {
                left_vector.assign(points.begin(), points.begin() + middle_indx);
            }

            std::vector<Node<PointT>> right_vector;
            if (middle_indx + 1 < points.size()) {
                right_vector.assign(points.begin() + middle_indx + 1, points.end());
            }

            if (!left_vector.empty())
                points_to_be_proccessed->push({left_vector, new_axis});
            
            if (!right_vector.empty())
                points_to_be_proccessed->push({right_vector, new_axis});
        }
        
    }

    /*
    Input:
    @cloud => cloud data

    Return:
    void

    Function:
    This main function receives a point cloud, preprocess the data, and uses the data to then construct the tree by calling another function
    */
    template <typename PointT>
    void KdTree<PointT>::setTree(typename pcl::PointCloud<PointT>::Ptr cloud) {
        root = nullptr;
        int cloud_size = cloud->points.size();
        std::vector<Node<PointT>> indx_point_cloud(cloud_size);
        
        for (int indx = 0; indx < cloud_size; indx++) {
            PointT point = cloud->points[indx];
            indx_point_cloud[indx] = {point, indx};
        }

        setTree(indx_point_cloud);
    }

    template <typename PointT> 
    void KdTree<PointT>::traverse(Node<PointT>* &node, PointT target, float distanceTol, std::vector<int> &ids, int level) {
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
        
        // you go left when minAxes is left. why? because imagine, if are comapring xs, you will go left if the left part of the boudnary is less than current value, and then go also right if maxAxes of boudary is greater than your current value. 
        bool go_left = (target[indx] - distanceTol) < node->point[indx];
        bool go_right = (target[indx] + distanceTol) > node->point[indx];
        
        level += 1; // increase level for next call
        
        // Just need to check if is within box, to add it. If not, just omit and continue with code
        if (within_box) {
            float distance = sqrt( (target_x - curr_x)*(target_x - curr_x) + (target_y - curr_y)*(target_y - curr_y));
            
            // check if distance is within distance tolerance
            if (distance <= distanceTol)
            ids.push_back(curr_id);
        }
        
        // Both of the conditionals are independent. Can occur that both get executed (when within boundaries) or just one of them
        
        if (go_left) { // if only need to go left
            traverse(node->left, target, distanceTol, ids, level);
        } 
        
        if (go_right) { // if only need to go and check right boundary
            traverse(node->right, target, distanceTol, ids, level);
        }
    }
		
    // return a list of point ids in the tree that are within distance of target
    template <typename PointT> 
    std::vector<PointT> KdTree<PointT>::search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        traverse(root, target, distanceTol, ids, 0);
        return ids;
    }
		
}