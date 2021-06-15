/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac3d.cpp"
#include "cluster.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        //renderHighway(viewer);
        //egoCar.render(viewer);
        //car1.render(viewer);
        //car2.render(viewer);
        //car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
  	Lidar* lidar = new Lidar(cars, 0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor and segment the ground
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  	std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
  	//renderPointCloud(viewer, segmentCloud.first, "cloud_obstacles", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "cloud_road", Color(1,0,0));

    // Clustering the object above the ground
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessor.BoundingBox(*cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    
}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)

{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // Filter the input cloud by making a voxel grid and delimitating the region of interest (ROI)
    Eigen::Vector4f minPoint (-15,-6,-5,1);
    Eigen::Vector4f maxPoint (15,6,5, 1); 
    float filterRes = 0.3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud,filterRes,minPoint,maxPoint);
    
    // Segment the ground planes
  	std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudGround = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloudGround.second, "Ground point cloud", Color(0,1,0));

    // Clustering the object above the ground
    float clusterTolerance = 0.5;
    int minSize = 5;
    int maxSize = 300;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloudGround.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // Render each point cloud object with its bounding box
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessorI->BoundingBox(*cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

    //renderPointCloud(viewer,filteredCloud,"filteredCloud");
}

void cityBlock_fromScratch(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)

{
    // Filter the input cloud by making a voxel grid and delimitating the region of interest (ROI)
    Eigen::Vector4f minPoint (-15,-5,-5,1);
    Eigen::Vector4f maxPoint (15,5,5, 1); 
    float filterRes = 0.3;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud,filterRes,minPoint,maxPoint);

    // Segment road plane
    std::unordered_set<int> inliers = Ransac(filteredCloud, 10, 0.5);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudObjects(new pcl::PointCloud<pcl::PointXYZI>());
	for(int index = 0; index < filteredCloud->points.size(); index++)
	{
		pcl::PointXYZI point = filteredCloud->points[index];
		if(inliers.count(index))
			cloudPlane->points.push_back(point);
		else
			cloudObjects->points.push_back(point);
	}

    // Find clusters
    std::vector<std::vector<int>> clusters_index = getClusters(cloudObjects, 1.0);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
    for(std::vector<int> cluster_i : clusters_index)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZI>);
  		for(int indice: cluster_i)
  			clusterCloud->push_back(cloudObjects->points[indice]);			
		clusters.push_back(clusterCloud);
  	}

	// Render 3D point cloud with plane, obsjects and its bounding boxes
	if(inliers.size())
		renderPointCloud(viewer,cloudPlane,"inliers",Color(0,1,0));
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        renderPointCloud(viewer, cluster,"cluster"+std::to_string(clusterId),Color(1,0,0));
        pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
        pcl::copyPointCloud(*cluster, cloud_xyz);
        BoxQ box = pointProcessorI->MOBoundingBox(cloud_xyz);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }   
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock_fromScratch(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();

    } 
}