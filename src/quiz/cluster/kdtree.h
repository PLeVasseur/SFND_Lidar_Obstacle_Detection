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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, point, id, 0);
	}

	void insertHelper(Node **node, std::vector<float> point, int id, int dimension)
	{
		// recursive function that inserts the point into the kdtree
		if (NULL == *node) {
			*node = new Node(point, id);
		}
		else {
			// calculate current dimension
			int cd = dimension % 2;
			double split = (*node)->point[cd];

			if (point[cd] < split) {
				insertHelper(&((*node)->left), point, id, dimension + 1);
			} else {
				insertHelper(&((*node)->right), point, id, dimension + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, distanceTol, 0, ids, root);

		return ids;
	}

	void searchHelper(std::vector<float> target, float distanceTol, uint dim, std::vector<int> &nearbyIDs, Node *node)
	{
		if (NULL == node)
		{ 
			return;
		}
		else {
			double r_x = target[0] + distanceTol;
			double l_x = target[0] - distanceTol;
			double t_y = target[1] + distanceTol;
			double b_y = target[1] - distanceTol;

			std::vector<float> pt = node->point;

			// check if the point is nearby, i.e. within box
			if (pt[0] <= r_x && pt[0] >= l_x
			    && pt[1] <= t_y && pt[1] >= b_y) {
				// in the box, now check distance to see if it's nearby and should be added to list
				double euclid_dist = sqrt( pow((target[0] - pt[0]), 2) + pow((target[1] - pt[1]), 2));
				if (euclid_dist <= distanceTol) { 
					nearbyIDs.push_back( node->id );
				}
			}

			// check the regions of the kdtree we need to explore further
			uint cd = dim % 2; // 0 if x, 1 if y
			if (target[cd] - distanceTol < node->point[cd]) {
				searchHelper(target, distanceTol, dim + 1, nearbyIDs, node->left);
			}
			if (target[cd] + distanceTol > node->point[cd]) {
				searchHelper(target, distanceTol, dim + 1, nearbyIDs, node->right);
			}
		}
	}

};




