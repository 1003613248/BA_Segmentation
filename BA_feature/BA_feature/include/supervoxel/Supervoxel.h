#pragma once

#ifndef _SUPERVOXEL_H_
#define _SUPERVOXEL_H_


#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>
#include <fstream>
 
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>


#include <pcl/features/pfh.h>

#include "../utils/types.h"


using namespace std;

//parameters for supervoxel
typedef struct 
{
	float voxel_resolution;
	float seed_resolution;
	bool  use_transform;
	float color_importance;
	float spatial_importance;
	float normal_importance;

} SuperVoxelParameters ;

/*
 * This is a function to initialize the supervoxel structure and return the useful structures 
 * In:  pcloud_in , parameters: voxel_resolution : 0.05 seed_resolution: 0.5
 * Out: supervoxels: it contains the indexes of the pcloud_in which belongs to the same supervoxel label
 *      supervoxel_clusters: it contains the supervoxel structure of the pcloud_in which belongs to the same supervoxel label
 *      supervoxel_adjacency: it contains the adjacencies of the specific label which belongs to the same supervoxel label
 */    
void supervoxels_init(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_in,SuperVoxelParameters parameters,std::map <uint32_t, vector<uint32_t>> &supervoxels,
	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > & supervoxel_clusters ,std::multimap<uint32_t, uint32_t>& supervoxel_adjacency );


/*
 * This is a function to get the indexes of the points of orignal point cloud which belongs to the same supervoxel
 */    
void points_of_supervoxel(std::map <uint32_t, vector<uint32_t>> &supervoxels , uint32_t label ,vector<uint32_t>& points);
/*
 * This is a function to get the actual centroid infomation of a specific supervoxel
 */   
bool centroids_of_supervoxel( std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > & supervoxel_clusters , uint32_t label, pair<uint32_t,pcl::PointXYZRGBA>& centroid) ; 

/*
 * This is a function to get the adjacency supervoxel of a specific supervoxel
 */
vector<uint32_t> adjacency_of_supervoxel(std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,uint32_t label, pair<uint32_t, vector<uint32_t>>& adjacency);


class SupervoxelUnit
{
public:
	SupervoxelUnit(){};

	SupervoxelUnit(const SupervoxelUnit& c)
	{
		id = c.id ;
		point.x = c.point.x ;
		point.y = c.point.y ;
		point.z = c.point.z ;
		point.r = c.point.r ;
		point.g = c.point.g ;
		point.b = c.point.b ;
		point.label = c.point.label ;
		super_id = c.super_id ; 
		indices.clear();
		for (int i = 0 ; i < c.indices.size() ; i ++ )
		{
			indices.push_back( c.indices[i] );
		}
	};
	
	~SupervoxelUnit(){};

public :
	int super_id ; // unuse
	int id ; 
	PointType point ; 
	vector<uint32_t> indices ; 
};


class SupervoxelHelper
{
public:
	SupervoxelHelper(){}

	~SupervoxelHelper(){}

	vector<SupervoxelUnit> generateSupervoxel(const PointCloud& cloud);

public:
	SupervoxelUnit uints_;

private:
	//supervoxel
	std::map <uint32_t, vector<uint32_t>> _supervoxels;
	std::multimap<uint32_t, uint32_t> _supervoxel_adjacency;
};


#endif