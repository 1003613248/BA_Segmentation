#include "color.h"

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void MyColor::compute(const PointCloud& data,
					 KdTree& data_kdtree,
					 const std::vector<const PointType*>& interest_pts,
					 std::vector<std::vector<float> >& results)
{
	size_t nbr_interest_pts = interest_pts.size();
	results.resize(nbr_interest_pts);

	//search the the radius neighboring mean color 
	for (int i = 0 ; i < interest_pts.size() ; i ++ )
	{
		vector<float> distances ; 
		vector<int> indices ;

		data_kdtree.radiusSearch( *interest_pts[i],radius_,indices,distances);

		double r = 0 ; 
		double g = 0 ;
		double b = 0 ;
		for (int j = 0 ; j < indices.size() ; j ++ )
		{
			r += data[indices[j]].r ;
			g += data[indices[j]].g ;
			b += data[indices[j]].b ;
		}

		r = r / indices.size() ; 
		g = g / indices.size() ; 
		b = b / indices.size() ; 

		r = r / 255.0 ; 
		g = g / 255.0 ; 
		b = b / 255.0 ; 

		results[i].push_back(r);
		results[i].push_back(g);
		results[i].push_back(b);
	}
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void MyColor::compute(const PointCloud& data,
					 KdTree& data_kdtree,
					 const std::vector<const vector<int>*>& interest_region_indices,
					 std::vector<std::vector<float> >& results)
{
	size_t nbr_interest_regions = interest_region_indices.size();
	results.resize(nbr_interest_regions);


	for (size_t i = 0 ; i < nbr_interest_regions ; i++)
	{
		int size = 0  ;

		double r = 0 ; 
		double g = 0 ;
		double b = 0 ;

		for (int j = 0 ; j <interest_region_indices[i]->size(); j ++  )
		{
			r += data[ (*interest_region_indices[i])[j]].r ;
			g += data[ (*interest_region_indices[i])[j]].g ;
			b += data[ (*interest_region_indices[i])[j]].b ;
		}

		r = r / interest_region_indices[i]->size() ; 
		g = g / interest_region_indices[i]->size() ; 
		b = b / interest_region_indices[i]->size() ; 

		r = r / 255.0 ; 
		g = g / 255.0 ; 
		b = b / 255.0 ; 

		results[i].push_back(r);
		results[i].push_back(g);
		results[i].push_back(b);
		
	}
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void MyColor::compute(const PointCloud& data,
					  KdTree& data_kdtree,
					  const vector<SupervoxelUnit>& supervoxels,
					  vector<vector<float> >& results)
{
	size_t nbr_interest_pts = supervoxels.size();
	results.resize(nbr_interest_pts);

	//search the the radius neighboring mean color 
	for (int i = 0 ; i < supervoxels.size() ; i ++ )
	{

		vector<uint32_t> indices  = supervoxels[i].indices ;

		double r = 0 ; 
		double g = 0 ;
		double b = 0 ;
		for (int j = 0 ; j < indices.size() ; j ++ )
		{
			r += data[indices[j]].r ;
			g += data[indices[j]].g ;
			b += data[indices[j]].b ;
		}

		r = r / indices.size() ; 
		g = g / indices.size() ; 
		b = b / indices.size() ; 

		r = r / 255.0 ; 
		g = g / 255.0 ; 
		b = b / 255.0 ; 

		results[i].push_back(r);
		results[i].push_back(g);
		results[i].push_back(b);
	}
}


void MyColor::compute(const PointCloud& data, 
					  KdTree& data_kdtree, 
					  const vector< vector<SupervoxelUnit>* > & supervoxels, 
					  vector<vector<float> >& results )
{
	size_t nbr_interest_regions = supervoxels.size();
	results.resize(nbr_interest_regions);


	for (size_t i = 0 ; i < nbr_interest_regions ; i++)
	{
		int size = 0  ;

		double r = 0 ; 
		double g = 0 ;
		double b = 0 ;

		for (int j = 0 ; j < supervoxels[i]->size(); j ++  )
		{
			for ( int k = 0 ; k < (*supervoxels[i])[j].indices.size() ; k ++ )
			{
				r += data[(*supervoxels[i])[j].indices[k]].r ; 
				g += data[(*supervoxels[i])[j].indices[k]].g ; 
				b += data[(*supervoxels[i])[j].indices[k]].b ; 
				size ++ ;
			}
		}

		r = r / size ; 
		g = g / size ; 
		b = b / size ; 

		r = r / 255.0 ; 
		g = g / 255.0 ; 
		b = b / 255.0 ; 

		results[i].push_back(r);
		results[i].push_back(g);
		results[i].push_back(b);
	}
}