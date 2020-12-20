#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>

class KdTreeNode
{
public:
	std::vector<double> point;
	int id;
	KdTreeNode* left;
	KdTreeNode* right;

	KdTreeNode(const std::vector<double>& arr, const int setId)
		:point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

class KdTree
{
private:
	KdTreeNode* root;

	void insertNode(KdTreeNode*& node, const std::vector<double>& point, const int id, const int p)
	{
		if (node == nullptr)
		{
			node = new KdTreeNode(point, id);
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

	void deleteNodes(const KdTreeNode* node)
	{
		if (node == nullptr)
		{
			return;
		}

		deleteNodes(node->left);
		deleteNodes(node->right);

		delete node;
	}

	double fabs(const double& v)
	{
		return v < 0 ? -v : v;
	}

	void searchNode(
		const KdTreeNode* node,
		const int p,
		const std::vector<double>& target,
		const double& distanceTol,
		std::vector<int>& ids)
	{
		if (node == nullptr)
		{
			return;
		}

		double targetDist = fabs(node->point[p] - target[p]);
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
	KdTree() : root(nullptr) {}

	~KdTree()
	{
		deleteNodes(root);
	}

	void insert(const std::vector<double>& point, const int id)
	{
		insertNode(root, point, id, 0);
	}

	void search(const std::vector<double>& target, const double& distanceTol, std::vector<int>& ids)
	{
		searchNode(root, 0, target, distanceTol, ids);
	}
	

};

#endif /* KDTREE_H_ */