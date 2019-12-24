/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> currentInliers;
	srand(time(NULL));

	// For max iterations
	for (int i = 0; i < maxIterations; i++) {

		// Randomly sample subset and fit line
		int ptOneIdx = rand() % cloud->size();
		int ptTwoIdx = rand() % cloud->size();
		while(ptTwoIdx == ptOneIdx) {
			ptTwoIdx = rand() % cloud->size();
		}

		// general equation for a line: Ax + By + C = 0
		// given two points (x1, y1) and (x2, y2) the form is:
		// (y1 - y2)x + (x2 - x1)y + (x1 * y2 - x2 * y1) = 0
		// i.e.
		// A = (y1 - y2)
		// B = (x2 - x1)
		// C = (x1 * y2 - x2 * y1)
		double A = cloud->points[ptOneIdx].y - cloud->points[ptTwoIdx].y;
		double B = cloud->points[ptTwoIdx].x - cloud->points[ptOneIdx].x;
		double C = cloud->points[ptOneIdx].x * cloud->points[ptTwoIdx].y
		         - cloud->points[ptTwoIdx].x * cloud->points[ptOneIdx].y;

		currentInliers.clear();
		// Measure distance between every point and fitted line
		for (int idx = 0; idx < cloud->points.size(); ++idx)
		{
			if (currentInliers.count(idx) > 0) 
			{ continue; }

			double pt_x = cloud->points[idx].x;
			double pt_y = cloud->points[idx].y;

			// If distance is smaller than threshold count it as inlier
			// distance d = | Ax + By + C | / sqrt(A^2 + B^2) for a point (x, y)
			double d = abs(A * pt_x + B * pt_y + C) / sqrt( pow(A, 2) + pow(B, 2) );
			if (d < distanceTol) {
				currentInliers.insert(idx);
			}
		}

		// check to see if we have more inliers with current set
		// if so, we'll take these inliers instead
		if (currentInliers.size() > inliersResult.size()) {
			inliersResult = currentInliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> currentInliers;
	srand(time(NULL));

	// For max iterations
	for (int loop = 0; loop < maxIterations; loop++) {

		// Randomly sample subset and fit line
		currentInliers.clear();
		while (currentInliers.size() < 3) {
			currentInliers.insert(rand() % cloud->size());
		}

		double x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = currentInliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// general equation for a plane: Ax + By + Cz + D = 0
		// given three points (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) the form is:
		// ix + jy + kz - (ix1 + jy1 + kz1) = 0
		// where
		// A = i
		// B = j
		// C = k
		// D = -(ix1 + jy1 + kz1)
		// and
		// i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1)
		// j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1)
		// k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)

		double i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		double j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		double k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		double A = i;
		double B = j;
		double C = k;
		double D = -(i * x1 + j * y1 + k * z1);

		// // Measure distance between every point and fitted plane
		for (int idx = 0; idx < cloud->points.size(); ++idx)
		{
			if (currentInliers.count(idx) > 0) 
			{ continue; }

			double pt_x = cloud->points[idx].x;
			double pt_y = cloud->points[idx].y;
			double pt_z = cloud->points[idx].z;

			// If distance is smaller than threshold count it as inlier
			// distance d = | Ax + By + C | / sqrt(A^2 + B^2) for a point (x, y)
			double d = abs(A * pt_x + B * pt_y + C * pt_z + D) / sqrt( pow(A, 2) + pow(B, 2) + pow(C, 2) );
			if (d < distanceTol) {
				currentInliers.insert(idx);
			}
		}

		// check to see if we have more inliers with current set
		// if so, we'll take these inliers instead
		if (currentInliers.size() > inliersResult.size()) {
			inliersResult = currentInliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
