/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input = lidar->scan();
    // renderPointCloud(viewer, input, "input");    
    // renderRays(viewer, lidar->position, input);

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds->SegmentPlane(input, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        processPointClouds->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle clusters"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = processPointClouds->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId ++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_input = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-50, -10, -10, 1), Eigen::Vector4f(60, 10, 10, 1));
    std::cout << "filtered input cloud size" << filtered_input->points.size() << endl;

    // first make sure the original code can work
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RANSAC3D(filtered_input, 100, 0.2);
 //   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filtered_input, 100, 0.2);

    std::cout << segmentCloud.first->points.size() << " points for the obstacles" << endl;
    std::cout << segmentCloud.second->points.size() << " points for the road" << endl;
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));  // road set to green looks better
//    renderPointCloud(viewer, segmentCloud.second, "obstacle", Color(1, 0, 0));  

    KdTree* tree = new KdTree;
    for(int i=0; i < segmentCloud.first->points.size(); i++)
    {
        tree->insert3D(segmentCloud.first->points[i], i);
    //    std::cout << "insert " << i << "point" << std::endl;
    }
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, tree, 0.35, 30, 300);
 //   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.second, 0.35, 3, 600);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    
    std::cout << "the total number of obstacles: " << cloudClusters.size() << std::endl;

    // rending host vehicle
    Box host = {-1.5, -1, -1, 2.5, 1, 1};
    renderBox(viewer, host, 99, Color(0.5, 0.5, 0.5), 0.8);
    int points = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        points += cluster->size();
        
        renderPointCloud(viewer, cluster, "obstacle clusters"+std::to_string(clusterId), colors[clusterId%colors.size()]);
     //   renderPointCloud(viewer, cluster, "obstacle clusters"+std::to_string(clusterId), Color( 0.5, 0.5, 1));
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        clusterId ++;
    }
    std::cout << "totla points " << points << std::endl; 
    // renderPointCloud(viewer, filtered_input, "Intensity");
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
//    simpleHighway(viewer);
 
    //for cityBlock
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // for cityBlock
    while (!viewer->wasStopped ())
    {
        //Clear Viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detectioni process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 

    // for simpleHighway
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 
}