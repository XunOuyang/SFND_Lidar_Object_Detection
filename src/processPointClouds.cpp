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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::Cropping(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, bool flag)
{
    typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setInputCloud(cloud);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud);
    region.setNegative(flag);
    region.filter(*result);

    return result;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::DownSampling(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes)
{
    typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
   pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    // LeafSize -- Grid Size
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*result);

    return result;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Downsampling
    typename pcl::PointCloud<PointT>::Ptr result = ProcessPointClouds<PointT>::DownSampling(cloud, filterRes);
    
    // Cropping:
    // Cropping step 1: crop out of range data
    result = ProcessPointClouds<PointT>::Cropping(result, minPoint, maxPoint, false);
    // Cropping step 2: remove the car top (hard code the vehicle top coordinates range)

    result = ProcessPointClouds<PointT>::Cropping(result, Eigen::Vector4f(-1.5, -1.7, -2, 1), Eigen::Vector4f(2.6, 1.7, 0, 1), true);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return result;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // RANSAC algo staring time
    auto startTime = std::chrono::steady_clock::now();

    // 3D RANSAC algo implementation:
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	while(maxIterations--) {
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
		double distance_2 = distanceThreshold*distanceThreshold;
		for(int index=0; index<cloud->points.size(); index++) {
            if(inliers.count(index) > 0)
                continue;
			double x = cloud->points[index].x;
			double y = cloud->points[index].y;
			double z = cloud->points[index].z;
			if( (A*x+B*y+C*z+D)*(A*x+B*y+C*z+D) <= distance_2 * (A*A+B*B+C*C))
				inliers.insert(index);
		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}

    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			road->points.push_back(point);
		else
			obstacles->points.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "road segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);
    std::cerr << " PointCloud representing the planar component: " << obstacles->width * obstacles->height << " data points. " << std::endl;


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.    

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Why pcl::PointIndices has not brackets. There are brackets in official documentation. from pointcloud extract_indices.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices() );
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients() );
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0) {
        std::cerr << " Could not estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& visited, KdTree* tree, float distanceTol)
{
    visited[index] = true;
    cluster.push_back(index);
    std::vector<int> nearest = tree->search3D(cloud->points[index], distanceTol);
    for(int id:nearest)
    {
        if(!visited[id])
            clusterHelper(id, cloud, cluster, visited, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> visited(cloud->points.size(), false);
    for(int i=0; i<cloud->points.size(); i++)
	{	
        if(!visited[i])
        {
            std::vector<int> cluster_ids;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
            clusterHelper(i, cloud, cluster_ids, visited, tree, clusterTolerance);
            if(cluster_ids.size() >= minSize && cluster_ids.size() <= maxSize)
            {
                for(int j=0; j<cluster_ids.size(); j++)
                {
                    PointT point;
                    point = cloud->points[cluster_ids[j]];
                    cloudCluster->points.push_back(point);
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                clusters.push_back(cloudCluster);
            }
            // else
            // {
            //     for(int k=1; k < cluster_ids.size(); k++)
            //     {
            //         visited[cluster_ids[k]] = false;
            //     }
            // }
        }      	
	} 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // By making use of the segmentation function above, we can get the road and obstacles

    typename pcl::PointCloud<PointT>::Ptr road;
    typename pcl::PointCloud<PointT>::Ptr obstacles;


    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    //     typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    //     for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
    //          cloud_cluster->points.push_back(cloud->points[*pit]);
    for(pcl::PointIndices getIndices: cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(int index: getIndices.indices) {
            cloud_cluster->points.push_back(cloud->points[index]);
        }
        cloud->width = cloud_cluster->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

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