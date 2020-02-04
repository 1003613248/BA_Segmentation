#ifndef _TYPES_H_
#define _TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>

class CPoint
{

public:
	CPoint(){}
	CPoint(const CPoint& p )
	{
		this->x = p.x;  
		this->y = p.y ; 
		this->z = p.z ; 
		this->label = p.label ; 
		this->index = p.index ;
	}
	CPoint(float x , float y , float z , int index, unsigned int label = 0 )
	{
		this->x = x;  
		this->y = y ; 
		this->z = z ; 
		this->label = label ; 
		this->index = index ;
	}
	~CPoint(){}

public:

	float x ;
	float y ;
	float z ;
	unsigned int label ;
	int index ;

};

class InformativeNode
{
public:
	float distance ; 
	int id ; 
	int scene_id ;
	float energy; 
	const float* feature;
	float label;
};

typedef pcl::PointXYZRGBL PointType ;

typedef pcl::PointCloud<PointType> PointCloud ;  

typedef pcl::KdTreeFLANN<PointType> KdTree ; 

#endif