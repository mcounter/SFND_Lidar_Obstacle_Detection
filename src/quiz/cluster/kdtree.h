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
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
private:
	void insertNode(Node*& node, const std::vector<float>& point, const int id, const int p)
	{
		if (node == nullptr)
		{
			node = new Node(point, id);
		}
		else if (point[p] < node->point[p])
		{
			insertNode(node->left, point, id, (p + 1) % point.size());
		}
		else
		{
			insertNode(node->right, point, id, (p + 1) % point.size());
		}
	}

	void deleteNodes(const Node* node)
	{
		if (node == nullptr)
		{
			return;
		}

		deleteNodes(node->left);
		deleteNodes(node->right);

		delete node;
	}

	float fabs(const float& v)
	{
		return v < 0 ? -v : v;
	}

	void searchNode(const Node* node, const int p,
					const std::vector<float>& target, const float& distanceTol,
					std::vector<int>& ids)
	{
		if (node == nullptr)
		{
			return;
		}

		float targetDist = fabs(node->point[p] - target[p]);
		int n = target.size();

		if (targetDist <= distanceTol)
		{
			double distanceTolSq = distanceTol * distanceTol;

			double dist = 0;
			for (int i = 0; i < n; ++i)
			{
				double v = node->point[i] - target[i];
				dist += v * v;
			}

			if (dist <= distanceTolSq)
			{
				ids.push_back(node->id);
			}
		}

		if (targetDist < distanceTol ||
			target[p] < node->point[p])
		{
			searchNode(node->left, (p + 1) % n, target, distanceTol, ids);
		}

		if (targetDist < distanceTol ||
			target[p] >= node->point[p])
		{
			searchNode(node->right, (p + 1) % n, target, distanceTol, ids);
		}
	}

public:
	Node* root;

	KdTree() : root(nullptr) {}

	~KdTree()
	{
		deleteNodes(root);
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(root, point, id, 0);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, const float& distanceTol)
	{
		std::vector<int> ids;

		searchNode(root, 0, target, distanceTol, ids);

		return ids;
	}
	

};




