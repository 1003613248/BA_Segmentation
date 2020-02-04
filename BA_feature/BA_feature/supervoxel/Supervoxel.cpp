#include "Supervoxel.h"

void supervoxels_init(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud_in,SuperVoxelParameters parameters,std::map <uint32_t, vector<uint32_t>> &supervoxels,
	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > & supervoxel_clusters ,std::multimap<uint32_t, uint32_t>& supervoxel_adjacency )
{
	//get the xyz information from the origin cloud

	pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (parameters.voxel_resolution,parameters.seed_resolution,parameters.use_transform);
	super.setInputCloud ( pcloud_in );
	super.setColorImportance (    parameters.color_importance );
	super.setSpatialImportance (  parameters.spatial_importance );
	super.setNormalImportance (   parameters.normal_importance );

	super.extract (supervoxel_clusters) ;

	cout << "supervoxel_cluster size: " << supervoxel_clusters.size() << endl ;

	pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = super.getLabeledCloud();

	cout << labeled_cloud->points.size() << endl ;
	supervoxels.clear(); 

	for (int i=0;i<labeled_cloud->points.size();i++)
	{
		uint32_t label=labeled_cloud->points[i].label;

		std::map <uint32_t, vector<uint32_t>>::iterator it = supervoxels.find(label);
		if( it == supervoxels.end() )
		{
			vector<uint32_t> points; 
			points.push_back(i);
			supervoxels.insert(map<uint32_t, vector<uint32_t>>::value_type( label ,points )) ; 

		}
		else
		{
			it->second.push_back(i);
		}
	}

#ifdef _DEBUG
	cout << "Getting supervoxel adjacency" << endl ;
#endif
	super.getSupervoxelAdjacency (supervoxel_adjacency);

}

void points_of_supervoxel(std::map <uint32_t, vector<uint32_t>> &supervoxels , uint32_t label ,vector<uint32_t>& points)
{
	points.clear(); 

	map<uint32_t, vector<uint32_t>>::iterator it = supervoxels.find(label);
	if (it!=supervoxels.end())
	{
		for (int i = 0 ; i < it->second.size() ; i ++ )
		{
			points.push_back(it->second[i]);
		}
	}
	else
	{
		cerr << "cannot find the label " << label << endl ;
	}
}

bool centroids_of_supervoxel( std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > & supervoxel_clusters , uint32_t label, pair<uint32_t,pcl::PointXYZRGBA>& centroid)
{
	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator it = supervoxel_clusters.find(label);

	if (it!=supervoxel_clusters.end())
	{
		pcl::PointXYZRGBA centroid_t = it->second->centroid_;
		centroid = make_pair(label,centroid_t);
		return true  ;
	}
	else
	{
		return false ;
	}


}

std::vector<uint32_t> adjacency_of_supervoxel(std::multimap<uint32_t, uint32_t>& supervoxel_adjacency,uint32_t label, pair<uint32_t, vector<uint32_t>>& adjacency)
{
	std::vector<uint32_t> labels ; 

	std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (label).first;
	for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (label).second; ++adjacent_itr)
	{
		labels.push_back(adjacent_itr->second);
	}
	adjacency = make_pair(label,labels);

	return labels; 
}

vector<SupervoxelUnit> SupervoxelHelper::generateSupervoxel(const PointCloud& cloud)
{
	vector<SupervoxelUnit> units; 

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for (int i = 0 ; i < cloud.points.size() ; i ++ )
	{
		pcl::PointXYZRGBA point ; 

		point.x = cloud.points[i].x ; 
		point.y = cloud.points[i].y ;
		point.z = cloud.points[i].z ;
		point.r = cloud.points[i].r ;
		point.g = cloud.points[i].g ;
		point.b = cloud.points[i].b ;
		pcloud->points.push_back(point);
	}


	//parameters for supervoxel
	SuperVoxelParameters superParam ; 
	superParam.color_importance =	0.3 ; 
	superParam.normal_importance =	0.3;
	superParam.spatial_importance = 0.1; 
	superParam.seed_resolution =	0.3 ;
	superParam.voxel_resolution =	0.1 ; 
	superParam.use_transform = false;


	std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
	supervoxels_init(pcloud,superParam,_supervoxels,supervoxel_clusters,_supervoxel_adjacency);

	for (std::map <uint32_t, vector<uint32_t>>::iterator it = _supervoxels.begin() ; it != _supervoxels.end();  it++  )
	{
		SupervoxelUnit unit ; 

		unit.id = it->first ;

		PointCloud::Ptr subcloud(new PointCloud) ;

		PointType centroid;

		centroid.x = 0 ; 
		centroid.y = 0 ; 
		centroid.z = 0 ;

		for (int pix = 0 ; pix < it->second.size() ; pix ++ )
		{
			PointType point ;
			point.x = cloud.points[it->second[pix]].x;
			point.y = cloud.points[it->second[pix]].y;
			point.z = cloud.points[it->second[pix]].z;
			point.r = cloud.points[it->second[pix]].r;
			point.g = cloud.points[it->second[pix]].g;
			point.b = cloud.points[it->second[pix]].b;

			centroid.x += point.x ;
			centroid.y += point.y ;
			centroid.z += point.z ;

			point.label = cloud.points[it->second[pix]].label;

			subcloud->points.push_back(point);
			unit.indices.push_back(it->second[pix]);
		}

		centroid.x = centroid.x / subcloud->points.size();
		centroid.y = centroid.y / subcloud->points.size();
		centroid.z = centroid.z / subcloud->points.size();

		KdTree tree ; 
		tree.setInputCloud(subcloud);

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if (tree.nearestKSearch(centroid, 1 , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			unit.point.x = subcloud->points[pointIdxRadiusSearch[0]].x ;
			unit.point.y = subcloud->points[pointIdxRadiusSearch[0]].y ;
			unit.point.z = subcloud->points[pointIdxRadiusSearch[0]].z ;
			unit.point.r = subcloud->points[pointIdxRadiusSearch[0]].r ;
			unit.point.g = subcloud->points[pointIdxRadiusSearch[0]].g ;
			unit.point.b = subcloud->points[pointIdxRadiusSearch[0]].b ;
			unit.point.label = subcloud->points[pointIdxRadiusSearch[0]].label ;
		}
		else
		{
			cout << "error to compute centroid " << endl ; 
			exit(-1);
		}

		units.push_back(unit);
	}

	return units;
}
