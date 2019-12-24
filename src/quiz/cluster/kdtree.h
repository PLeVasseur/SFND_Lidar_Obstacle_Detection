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
		return ids;
	}
	

};




