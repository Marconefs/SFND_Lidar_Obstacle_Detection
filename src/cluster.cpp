/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

//#include "render/render.h"
//#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

template<typename PointT>
void clusterHelper(int indice, PointT points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<float> point = {points->points[indice].x, points->points[indice].y, points->points[indice].z};
	std::vector<int> nearest = tree->search(point, distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(PointT points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points->points.size(), false);
 
	int i = 0;
	while(i < points->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;

}


template<typename PointT> 
std::vector<std::vector<int>> getClusters (PointT cloud, float distanceTol)
{

	KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++)
	{
		std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    	tree->insert(point,i); 
	}

  	std::vector<std::vector<int>> clusters_index = euclideanCluster(cloud, tree, distanceTol);

	return clusters_index;
}
