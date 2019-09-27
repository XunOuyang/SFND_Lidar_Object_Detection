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
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	
	while(maxIterations--) {
		// // 2D
		// int i = rand()%cloud->points.size();
		// int j = rand()%cloud->points.size();
		// double x1 = cloud->points[i].x;
		// double x2 = cloud->points[j].x;
		// double y1 = cloud->points[i].y;
		// double y2 = cloud->points[j].y;
		// double A = y1 - y2;
		// double B = x2 - x1;
		// double C = (x1*y2 - x2*y1);
		// std::unordered_set<int> inliers;
		// inliers.insert(i);
		// inliers.insert(j);
		// double distance_2 = distanceTol*distanceTol;
		// for(int k=0; k<cloud->points.size(); k++) {
		// 	double x = cloud->points[k].x;
		// 	double y = cloud->points[k].y;
		// 	if((A*x+B*y+C)*(A*x+B*y+C)/(A*A+B*B) <= distance_2) {
		// 		inliers.insert(k);
		// 	}
		// }
		// if(inliers.size() > inliersResult.size())
		// 	inliersResult = inliers;

		// 3D 
		int i = rand()%cloud->points.size();
		int j = rand()%cloud->points.size();
		int k = rand()%cloud->points.size();
		double x1 = cloud->points[i].x;
		double x2 = cloud->points[j].x;
		double x3 = cloud->points[k].x;
		double y1 = cloud->points[i].y;
		double y2 = cloud->points[j].y;
		double y3 = cloud->points[k].y;
		double z1 = cloud->points[i].z;
		double z2 = cloud->points[j].z;
		double z3 = cloud->points[k].z;
		double A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		double B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		double C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		double D = -(A*x1 + B*y1 + C*z1);
		std::unordered_set<int> inliers;
		inliers.insert(i);
		inliers.insert(j);
		inliers.insert(k);
		double distance_2 = distanceTol*distanceTol;
		for(int index=0; index<cloud->points.size(); index++) {
			double x = cloud->points[index].x;
			double y = cloud->points[index].y;
			double z = cloud->points[index].z;
			if( (A*x+B*y+C*z+D)*(A*x+B*y+C*z+D) <= distance_2 * (A*A+B*B+C*C))
				inliers.insert(index);
		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

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
	std::unordered_set<int> inliers = Ransac(cloud, 20, 0.2);

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
