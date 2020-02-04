#include "height.h"

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Height::compute(const PointCloud& data,
					 KdTree& data_kdtree,
					 const std::vector<const PointType*>& interest_pts,
					 std::vector<std::vector<float> >& results)
{
	size_t nbr_interest_pts = interest_pts.size();
	results.resize(nbr_interest_pts);

	//search the lowest point for the region
	PointType pointMax,pointMin ; 
	pointMax.x = -FLT_MAX ;  
	pointMax.y = -FLT_MAX ;
	pointMax.z = -FLT_MAX ; 
	pointMin.x = FLT_MAX ; 
	pointMin.y = FLT_MAX ; 
	pointMin.z = FLT_MAX ;

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		if (pointMax.x < data.points[i].x)
		{
			pointMax.x = data.points[i].x ;
		}
		if (pointMax.y < data.points[i].y)
		{
			pointMax.y = data.points[i].y ;
		}
		if (pointMax.z < data.points[i].z)
		{
			pointMax.z = data.points[i].z ;
		}
		if (pointMin.x > data.points[i].x)
		{
			pointMin.x = data.points[i].x ;
		}
		if (pointMin.y > data.points[i].y)
		{
			pointMin.y = data.points[i].y ;
		}
		if (pointMin.z > data.points[i].z)
		{
			pointMin.z = data.points[i].z ;
		}
	}

	int x_num = (pointMax.x - pointMin.x)/scale_ +1;
	int y_num = (pointMax.y - pointMin.y)/scale_ +1;
	vector<double*> lowest ;
	lowest.resize(x_num);
	for (int i = 0 ; i < x_num ; i ++ )
	{
		lowest[i] = new double[y_num];
		for ( int j = 0 ; j < y_num ; j ++ )
		{
			lowest[i][j] = 100000 ; 
		}
	}

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		int x_pos = (data.points[i].x - pointMin.x)/scale_ ;
		int y_pos = (data.points[i].y - pointMin.y)/scale_ ;

		if (data.points[i].z < lowest[x_pos][y_pos])
		{
			lowest[x_pos][y_pos] = data.points[i].z ;
		}
	}
	


	for ( size_t i = 0 ; i < nbr_interest_pts ; i++ )
	{
		results[i].resize(1);
		int x_pos = ((interest_pts[i])->x- pointMin.x)/scale_ ;
		int y_pos = ((interest_pts[i])->y- pointMin.y)/scale_ ;
		results[i][0] = ( (interest_pts[i])->z - lowest[x_pos][y_pos]  ) / 40.0 ;
	}
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Height::compute(const PointCloud& data,
					 KdTree& data_kdtree,
					 const std::vector<const vector<int>*>& interest_region_indices,
					 std::vector<std::vector<float> >& results)
{
	size_t nbr_interest_regions = interest_region_indices.size();
	results.resize(nbr_interest_regions);


	//search the lowest point for the region
	PointType pointMax,pointMin ; 
	pointMax.x = -100000 ;  
	pointMax.y = -100000 ;
	pointMax.z = -100000 ; 
	pointMin.x = 1000000 ; 
	pointMin.y = 1000000 ; 
	pointMin.z = 1000000 ;

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		if (pointMax.x < data.points[i].x)
		{
			pointMax.x = data.points[i].x ;
		}
		if (pointMax.y < data.points[i].y)
		{
			pointMax.y = data.points[i].y ;
		}
		if (pointMax.z < data.points[i].z)
		{
			pointMax.z = data.points[i].z ;
		}
		if (pointMin.x > data.points[i].x)
		{
			pointMin.x = data.points[i].x ;
		}
		if (pointMin.y > data.points[i].y)
		{
			pointMin.y = data.points[i].y ;
		}
		if (pointMin.z > data.points[i].z)
		{
			pointMin.z = data.points[i].z ;
		}
	}

	scale_ = 2 ; 
	int x_num = (pointMax.x - pointMin.x)/scale_ +1;
	int y_num = (pointMax.y - pointMin.y)/scale_ +1;
	vector<double*> lowest ;
	lowest.resize(x_num);
	for (int i = 0 ; i < x_num ; i ++ )
	{
		lowest[i] = new double[y_num];
		for ( int j = 0 ; j < y_num ; j ++ )
		{
			lowest[i][j] = 100000 ; 
		}
	}

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		int x_pos = (data.points[i].x - pointMin.x)/scale_ ;
		int y_pos = (data.points[i].y - pointMin.y)/scale_ ;

		if (data.points[i].z < lowest[x_pos][y_pos])
		{
			lowest[x_pos][y_pos] = data.points[i].z ;
		}
	}

	PointType region_centroid;
	for (size_t i = 0 ; i < nbr_interest_regions ; i++)
	{
		cloud_geometry::nearest::computeCentroid(data, *(interest_region_indices[i]), region_centroid);
		results[i].resize(1);

		int x_pos = (region_centroid.x- pointMin.x)/scale_ ;
		int y_pos = (region_centroid.y- pointMin.y)/scale_ ;

		results[i][0] = ( region_centroid.z - lowest[x_pos][y_pos] ) / 40.0 ;
	}
}


// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Height::compute(	const PointCloud& data, 
						KdTree& data_kdtree, 
						const vector<SupervoxelUnit>& supervoxels, 
						vector<vector<float> >& results )
{
	size_t nbr_interest_pts = supervoxels.size();

	results.resize(nbr_interest_pts);

	//search the lowest point for the region
	PointType pointMax,pointMin ; 
	pointMax.x = -FLT_MAX ;  
	pointMax.y = -FLT_MAX ;
	pointMax.z = -FLT_MAX ; 
	pointMin.x = FLT_MAX ; 
	pointMin.y = FLT_MAX ; 
	pointMin.z = FLT_MAX ;

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		if (pointMax.x < data.points[i].x)
		{
			pointMax.x = data.points[i].x ;
		}
		if (pointMax.y < data.points[i].y)
		{
			pointMax.y = data.points[i].y ;
		}
		if (pointMax.z < data.points[i].z)
		{
			pointMax.z = data.points[i].z ;
		}
		if (pointMin.x > data.points[i].x)
		{
			pointMin.x = data.points[i].x ;
		}
		if (pointMin.y > data.points[i].y)
		{
			pointMin.y = data.points[i].y ;
		}
		if (pointMin.z > data.points[i].z)
		{
			pointMin.z = data.points[i].z ;
		}
	}

	int x_num = (pointMax.x - pointMin.x)/scale_ +1;
	int y_num = (pointMax.y - pointMin.y)/scale_ +1;

	vector<double*> lowest ;
	lowest.resize(x_num);
	for (int i = 0 ; i < x_num ; i ++ )
	{
		lowest[i] = new double[y_num];
		for ( int j = 0 ; j < y_num ; j ++ )
		{
			lowest[i][j] = 100000 ; 
		}
	}

	for (int i =0 ; i < data.points.size() ;i ++ )
	{
		int x_pos = (data.points[i].x - pointMin.x)/scale_ ;
		int y_pos = (data.points[i].y - pointMin.y)/scale_ ;

		if (data.points[i].z < lowest[x_pos][y_pos])
		{
			lowest[x_pos][y_pos] = data.points[i].z ;
		}
	}



	for ( size_t i = 0 ; i < nbr_interest_pts ; i++ )
	{
		results[i].resize(1);
		int x_pos = (supervoxels[i].point.x- pointMin.x)/scale_ ;
		int y_pos = (supervoxels[i].point.y- pointMin.y)/scale_ ;
		results[i][0] = ( supervoxels[i].point.z - lowest[x_pos][y_pos]  ) / 40.0 ;
//		cout << i << " " << results[i][0] ;
//		cout << endl ; 
	}
}


// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void Height::compute(const PointCloud& data, 
					 KdTree& data_kdtree, 
					 const vector< vector<SupervoxelUnit>* > & supervoxels, 
					 vector<vector<float> >& results)
{
	cout << "not finish" << endl ; 
	exit(-1);
}
