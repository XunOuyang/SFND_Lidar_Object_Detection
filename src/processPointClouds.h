// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	// Be very careful about passing the node!!!
	void insertHelper(pcl::PointXYZI point, Node*& node, int depth, int id) {
		if(node == NULL) 
		{
			node = new Node(point, id);
		}
		else if(depth%3 == 1)
        {
            if(point.y <= node->point.y) 
            {
                insertHelper(point, node->left, depth+1, id);
            }
            else
            {
                insertHelper(point, node->right, depth+1, id);
            }	
        }
        else if(depth%3 == 0)
        {
            if(point.x <= node->point.x) 
            {
                insertHelper(point, node->left, depth+1, id);
            }
            else
            {
                insertHelper(point, node->right, depth+1, id);
            }	
        }
        else
        {
            if(point.z <= node->point.z) 
            {
                insertHelper(point, node->left, depth+1, id);
            }
            else
            {
                insertHelper(point, node->right, depth+1, id);
            }	
        }
        
        
	}
		
	void insert3D(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(point, root, 0, id);
	}

	void searchHelper(pcl::PointXYZI target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
        if(node != NULL)
		{
			if( (node->point.x >= (target.x - distanceTol)) && 
                (node->point.x <= (target.x + distanceTol)) && 
                (node->point.y >= (target.y - distanceTol)) && 
                (node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol)) && 
                (node->point.z <= (target.z + distanceTol)) )
			{
			 	float distance_2 = 
                        (node->point.x - target.x) * (node->point.x - target.x) + 
                        (node->point.y - target.y) * (node->point.y - target.y) +
                        (node->point.z - target.z) * (node->point.z - target.z);
			 	if(distanceTol*distanceTol >= distance_2)
			 		ids.push_back(node->id);							
			}

            if(depth % 3 == 0)
            {            
                if ((target.x - distanceTol) < node->point.x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.x + distanceTol) > node->point.x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            } 
            else if(depth % 3 == 1)
            {
                if ((target.y - distanceTol) < node->point.y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.y + distanceTol) > node->point.y)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            else
            {
                if ((target.z - distanceTol) < node->point.z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.z + distanceTol) > node->point.z)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);            
            }
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr Cropping(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, bool flag);

    typename pcl::PointCloud<PointT>::Ptr DownSampling(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RANSAC3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void clusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& visited, KdTree* tree, float distantTol);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */