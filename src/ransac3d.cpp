#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker

template <typename PointT>
std::unordered_set<int> Ransac(PointT cloud, int maxIterations, float distanceTol)
{
	/* 
	Randomly sample subset and fit plane
	Measure distance between every point and fitted plane
	If distance is smaller than threshold count it as inlier
	Return indicies of inliers from fitted line with most inliers
	*/
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
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

		// plane equation: Ax+By+Cz+D=0
		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1+j*y1+k*z1);

		for(int index=0; index < cloud->points.size(); index++)
		{
			if(samples.count(index)>0)
				continue;
			
			float x4 = cloud->points[index].x;
			float y4 = cloud->points[index].y;
			float z4 = cloud->points[index].z;
			
			float d = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);

			if(d <= distanceTol)
				samples.insert(index);
		}

		if(samples.size()>inliersResult.size())
		{
			inliersResult = samples;
		}
	}
	
	return inliersResult;

}