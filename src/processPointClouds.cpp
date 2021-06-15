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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	// Create the filtering object, and tranform the point cloud into a voxel grid
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (filterRes, filterRes, filterRes);
	vg.filter (*cloudFiltered);

	// Crop the region of interest (ROI)
	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	// Crop the region corresponding to the car roof area
	std::vector<int> indices;
	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
	for(int point : indices)
		inliers->indices.push_back(point);

	// Exclude the car roof points from the filtered point cloud
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true); // set negative = true, so the points are going to be excluded
	extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles (new typename pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    extract.setNegative (true);
    extract.filter (*cloud_obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::SACSegmentation<PointT> seg;
  	
  	seg.setOptimizeCoefficients (true); //optional
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setDistanceThreshold (distanceThreshold);
  	seg.setMaxIterations(maxIterations);

    seg.setInputCloud (cloud);
  	seg.segment (*inliers, *coefficients);

   	if (inliers->indices.size () == 0)
    {
    	std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  	}
	
    // TODO:: Fill in this function to find inliers for the cloud.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<pcl::PointIndices> cluster_indices;
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance); // 2cm
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);

	for (pcl::PointIndices getIndices: cluster_indices)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

		for (int index : getIndices.indices)
			cloud_cluster->push_back (cloud->points[index]);

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
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT> cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(cluster, minPoint, maxPoint);

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
BoxQ ProcessPointClouds<PointT>::MOBoundingBox(typename pcl::PointCloud<pcl::PointXYZ> cluster)
{

	// Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
																					///    the signs are different and the box doesn't get correctly oriented in some cases.
	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
	projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
	projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(cluster, *cloudPointsProjected, projectionTransform);
	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZ minPoint, maxPoint;
	pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

	// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
	
	BoxQ box;
	box.bboxTransform = bboxTransform;
	box.bboxQuaternion = bboxQuaternion;
	box.cube_length = maxPoint.x - minPoint.x;
	box.cube_width = maxPoint.y - minPoint.y;
	box.cube_height = maxPoint.z - minPoint.z;

    return box;
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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--)
	{
		std::unordered_set<int> samples;
		while(samples.size()<3)
			samples.insert(rand()%(cloud->points.size()));

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = samples.begin();
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

		//cross product
		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		// equacao do plano Ax+By+Cz+D=0
		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1+j*y1+k*z1);

		for(int index=0; index < cloud->points.size(); index++)
		{
			// pula a posicao em ´cloud´ que foram retirados pelo sample
			if(samples.count(index)>0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			
			float d = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);

			if(d <= distanceTol)
				samples.insert(index);
		}

		if(samples.size()>inliersResult.size())
		{
			inliersResult = samples;
		}
	}
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}