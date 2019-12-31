#ifndef CLUSTER_H
#define CLUSTER_H

#include "kdtree.h"

void proximity(const std::vector<std::vector<float>>& points, float distanceTol, std::vector<int>& cluster, KdTree* tree, bool processed[], int searchPointID);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);
void testFunc();

#endif