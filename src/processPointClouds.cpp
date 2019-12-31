// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_roi (new pcl::PointCloud<PointT>);

    typename pcl::CropBox<PointT> roi;
    roi.setInputCloud(cloud_filtered);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.filter(*cloud_roi);

    std::vector<int> indices;

    typename pcl::CropBox<PointT> roof;
    roof.setInputCloud(cloud_roi);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1,   1));
    roof.setMax(Eigen::Vector4f( 2.6,  1.7, -0.4, 1));
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : indices)
    { inliers->indices.push_back(point); }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roi);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr groundPlane (new pcl::PointCloud<PointT>), objects (new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers (the ground plane)
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*groundPlane);

    // Extract the outliers (the obstacles)
    extract.setNegative (true);
    extract.filter (*objects);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(objects, groundPlane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// TODO: Has some issues; with the same parameters it looks like sometimes it picks all the points as ground plane
//       or something like that. All points go green, which indicates all points were call inliers by SegmentPlane2
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane2(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
	std::unordered_set<int> currentInliers;
	srand(time(NULL));

	// For max iterations
	for (int loop = 0; loop < maxIterations; loop++) {

		// Randomly sample subset and fit plane
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
			if (d < distanceThreshold) {
				currentInliers.insert(idx);
			}
		}

		// check to see if we have more inliers with current set
		// if so, we'll take these inliers instead
		if (currentInliers.size() > inliersResult.size()) {
			inliersResult = currentInliers;
		}
	}

	// Put inliers in a PointIndices
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    for (int index : inliersResult)
    { inliers->indices.push_back( index ); }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for (auto getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            { cloud_cluster->points.push_back (cloud->points[index]); }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering2(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO: insert from scratch Euclidean clustering
    KdTree* tree = new KdTree;

    // munging here to get point cloud data into format for euclidean clustering
    std::vector<std::vector<float>> points;
    for (int i=0; i<cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back( point );
        tree->insert(point,i);
    }
    // munging here to get point cloud data into format for euclidean clustering
    	
    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);

    // munging here to get indices back into PCL PointIndices for further use
    std::vector<pcl::PointIndices> clusterPointIndices;
    for (std::vector<int> clusterIndice : clusterIndices)
    {
        pcl::PointIndices clusterPointIndice;
        for (int clusterInd : clusterIndice)
        {
            clusterPointIndice.indices.push_back( clusterInd );
        }
        clusterPointIndices.push_back( clusterPointIndice );
    }
    // munging here to get indices back into PCL PointIndices for further use

    for (auto getIndices : clusterPointIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            { cloud_cluster->points.push_back (cloud->points[index]); }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxPCA(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions by
    // finding the centroid
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid); // had to derefence the boost shared pointer to
                                                   // match the function signature of compute3DCentroid
    // getting the normalized covariance matrix of the point cloud
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    // computing the eigenvectors of the normalized covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());

    // Only deal with rotation about z axis, not x or y axes, cars most likely only yawing not rolling or pitching
    // useful page for how to extract yaw from rotation matrix: http://planning.cs.uiuc.edu/node103.html
    double yaw = atan2(eigenVectorsPCA(1,0), eigenVectorsPCA(0,0));
    Eigen::Matrix3f yawOnly;
    yawOnly(0,0) = cos(yaw);
    yawOnly(0,1) = -sin(yaw);
    yawOnly(1,0) = sin(yaw);
    yawOnly(1,1) = cos(yaw);
    yawOnly(2,2) = 1.f;
    
    projectionTransform.block<3,3>(0,0) = yawOnly.transpose(); // take transpose to undo the rotation
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(yawOnly); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = yawOnly * meanDiagonal + pcaCentroid.head<3>();

    BoxQ boxq;
    boxq.bboxTransform = bboxTransform;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}