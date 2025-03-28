#include "kdtree.h"
#include "pcl/point_cloud.h"
#include <algorithm>
#include <iterator>
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

    template <typename PointT>
    Node<PointT>::~Node() {}

    //* KdTree Implementation
    template <typename PointT>
    KdTree<PointT>::KdTree() : root(NULL) {}

    template <typename PointT>
    KdTree<PointT>::~KdTree() {
        deleteTree(root);
    }

    template <typename PointT>
    void KdTree<PointT>::deleteTree(Node<PointT>* node) {
        if (node == nullptr) return;
        
        deleteTree(node->left);
        deleteTree(node->right);
        
        delete node;
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
    bool KdTree<PointT>::firstPointGreater(PointT first_point, PointT second_point, int depth) {
        int axis_to_compare = depth % dimensions;

        switch (axis_to_compare) {
        case 0:
            return first_point.x > second_point.x;
        case 1:
            return first_point.y > second_point.y;
        case 2:
            return first_point.z > second_point.z;
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

    /**
     * @brief Function creates a tree given iterators of node vectors
     * 
     * Use iterators to avoid creating a queue of vector of O(n^2) space, to O(N) space
     *
     * @param first first iterator of vector
     * @param last last iterator of vector
     * @return template <typename PointT> void
     */
    template <typename PointT>
    template <class Iterator>
    void KdTree<PointT>::setTree(Iterator first, Iterator last) {
        std::queue<std::pair<std::pair<Iterator, Iterator>, int>> points_to_be_proccessed;
        points_to_be_proccessed.push({{first, last}, 0}); // Set first/last iterator, and set initial axis to be added

        //* To avoid recursion, we will use a queue to sort iteratively the vector, get the middle point, insert, and continue. In a way that we fill each time every level
        while (!points_to_be_proccessed.empty()) {
            auto [firstIt, lastIt] = points_to_be_proccessed.front().first;
            int axis = points_to_be_proccessed.front().second;
            points_to_be_proccessed.pop();

            //* This lambda function is a helper sort function to consider different axis (xyz)
            auto sort_by_axis = [axis] (Node<PointT> p1, Node<PointT> p2) {
            switch (axis)
            {
            case 0:
                return p1.point.x < p2.point.x;
            case 1:
                return p1.point.y < p2.point.y;
            case 2:
                return p1.point.z < p2.point.z;
            default:
                return false;
            }
            };


            
            //* Insert the middle point into the tree (to have a balanced tree)
            auto size = std::distance(firstIt, lastIt);
            int middle_indx = size / 2;
            
            //* nth_element just makes sure that the nth element of the array is the same as the nth element in the sorted array. this because we are only interested on middle element. 
            std::nth_element(firstIt, firstIt + middle_indx, lastIt, sort_by_axis);

            PointT middle_point = (*(firstIt + middle_indx)).point;
            int real_indx = (*(firstIt + middle_indx)).indx;

            insert(middle_point, real_indx);

            //* Update queue so in next iteration insert the values that must be
            int new_axis = (axis + 1) % dimensions; // Must use modulo because in each vector we want to iterate between x, y, and z. This helps to keep count

            //* Before creating the vectors, we need to validate that there are valid numbers to create them
            if (middle_indx > 0) 
                points_to_be_proccessed.push({{firstIt, firstIt + middle_indx}, new_axis});

            if (middle_indx + 1 < size) 
                points_to_be_proccessed.push({{firstIt + middle_indx + 1, lastIt}, new_axis});
            
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
    void KdTree<PointT>::setTree(const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
        root = nullptr;
        int cloud_size = cloud->points.size();
        std::vector<Node<PointT>> indx_point_cloud(cloud_size);
        
        for (int indx = 0; indx < cloud_size; indx++) {
            PointT point = cloud->points[indx];
            indx_point_cloud[indx] = {point, indx};
        }

        setTree(indx_point_cloud.begin(), indx_point_cloud.end());
    }

    /*
    Input: 
    @point => this is the point that we want to know if is within range of reference point given a distance tolerance
    @reference_point => our reference point which defines our surrounding permitted area

    Return:
    true/false => return true or false if is within range

    Function:
    This function receives a point and a reference point, and tells if the point is within distance_tol of the reference point
    */
    template <typename PointT>
    bool KdTree<PointT>::firstPointWithinRangeSecond(const PointT& curr_point, const PointT& ref_point) const {
        bool withinBoundariesX = true;
        bool withinBoundariesY = true;
        bool withinBoundariesZ = true;

        //* Check manually each dimensions if it exists, and then check if point is within boundaries
        if (dimensions > 0) {
            float lowerX = curr_point.x - distance_tol;
            float upperX = curr_point.x + distance_tol;
            withinBoundariesX = (lowerX <= ref_point.x) && (ref_point.x <= upperX);
        }

        if (dimensions > 1) {
            float lowerY = curr_point.y - distance_tol;
            float upperY = curr_point.y + distance_tol;
            withinBoundariesY = (lowerY <= ref_point.y) && (ref_point.y <= upperY);
        }

        if (dimensions == 3) {
            float lowerZ = curr_point.z - distance_tol;
            float upperZ = curr_point.z + distance_tol;
            withinBoundariesZ = (lowerZ <= ref_point.z) && (ref_point.z <= upperZ);
        }
        
        return withinBoundariesX && withinBoundariesY && withinBoundariesZ;
    }

    /*
    Inputs
    @1, p2 => Node points that we want to know if are withing a distance. 

    Returns:
    true/false => if the two points are within distance_tol (variable of the class)
    */
    template <typename PointT>
    bool KdTree<PointT>::withinDistance(const PointT& target_point, const PointT& source_point) const {
        float distance = 0.0;
        float dx, dy, dz;

        const float distanceTolSquared = distance_tol * distance_tol;
        //* Depending on the dimension, the distance function changes
        switch (dimensions) {
        case 3:
            dx = target_point.x - source_point.x;
            dy = target_point.y - source_point.y;
            dz = target_point.z - source_point.z;

            distance = (dx * dx) + (dy * dy) + (dz * dz);
            return distance <= distanceTolSquared;
        case 2:
            dx = target_point.x - source_point.x;
            dy = target_point.y - source_point.y;

            distance = (dx * dx) + (dy * dy);
            return distance <= distanceTolSquared;
        case 1:
            dx = target_point.x - source_point.x;

            distance = (dx * dx);
            return distance <= distanceTolSquared;
        default:
            return false;
        }
    }
    
		
    /*
    Input:
    @target => Point which we want to find all the nearest points
    
    Returns:
    @indices => returns a vector of indices

    Function:
    This function receives a point to find nearest points within a distance tolerance. It obtains the nearest points and then returns their original indices of the point cloud
    */ 
    template <typename PointT> 
    std::vector<int> KdTree<PointT>::search(const PointT& target) const {
        if (root == nullptr) return {};
        std::vector<int> indices;
        
        std::queue<std::pair<Node<PointT>*, int>> queue;
        queue.push({root, 0});

        //* Use a queue to traverse the entire tree
        while (!queue.empty()) {
            Node<PointT>* curr_point = queue.front().first;
            int level = queue.front().second;
            queue.pop();

            // bool within_range = firstPointWithinRangeSecond(curr_point->point, target);

            //* Depending on the dimensions that the point has, calculate if we need to go left/right
            bool traverse_left = false;
            bool traverse_right = false;
            int target_axis = level % dimensions;
            switch (target_axis) {
            case 0:
                traverse_left = (target.x - distance_tol) < curr_point->point.x;
                traverse_right = (target.x + distance_tol) > curr_point->point.x;
                break;
            case 1: 
                traverse_left = (target.y - distance_tol) < curr_point->point.y;
                traverse_right = (target.y + distance_tol) > curr_point->point.y;
                break;
            case 2:
                traverse_left = (target.z - distance_tol) < curr_point->point.z;
                traverse_right = (target.z + distance_tol) > curr_point->point.z;
                break;
            }

            if (withinDistance(target, curr_point->point))
                    indices.push_back(curr_point->indx);
            

            //* If we need to traverse the left/right side of the tree, then that node will be added to the queue
            if (traverse_left && curr_point->left != nullptr)
                queue.push({curr_point->left, level + 1});
        
            if (traverse_right && curr_point->right != nullptr)
                queue.push({curr_point->right, level + 1});

        }

        return indices;
    }
		
    /*
    Function that sets the entire distance tolerance of the tree
    */
    template <typename PointT>
    void KdTree<PointT>::setDistanceTol(float distance_tol) {
        this->distance_tol = abs(distance_tol);
    }
}