/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

void testFunc()
{}

void proximity(const std::vector<std::vector<float>>& points, float distanceTol, std::vector<int>& cluster, KdTree* tree, bool processed[], int searchPointID)
{
	processed[searchPointID] = true;
	cluster.push_back(searchPointID);
	std::vector<int> nearbyPoints = tree->search(points[searchPointID], distanceTol);

	for(int nearbyPoint : nearbyPoints) {
		if (processed[nearbyPoint] == false) {
			proximity(points, distanceTol, cluster, tree, processed, nearbyPoint);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;

	// ASSUMPTION: Order points passed into this function matches the order of points inserted into kdtree
	bool processed[points.size()] = {};

	for (int i = 0; i < points.size(); ++i) {
		if (processed[i] == false) {
			std::vector<int> cluster;
			proximity(points, distanceTol, cluster, tree, processed, i);
			if (cluster.size() > minSize && cluster.size() < maxSize)
			{ clusters.push_back(cluster); }
		}
	}
 
	return clusters;
}