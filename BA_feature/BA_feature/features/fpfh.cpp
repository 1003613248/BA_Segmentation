#include "fpfh.h"

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void FPFH::compute(const PointCloud& data,
				   KdTree& data_kdtree,
				   const std::vector<const PointType*>& interest_pts,
				   std::vector<std::vector<float> >& results)
{
	results.resize(interest_pts.size());

	vector<vector<int>> interest_region_neigbors_neigbors ; 
	interest_region_neigbors_neigbors.resize(interest_pts.size()); 

	for (int i = 0 ; i < interest_pts.size() ; i ++ )
	{
		vector<float> distances ; 
		vector<int> indices ;

		data_kdtree.radiusSearch( *interest_pts[i],_radius_fpfh,indices,distances);

		for (int j = 0 ; j < indices.size() ; j ++ )
		{
			interest_region_neigbors_neigbors[i].push_back(indices[j]);
		}
	}

	computeFeatures(data,data_kdtree,interest_region_neigbors_neigbors,results);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void FPFH::compute(const PointCloud& data,
				   KdTree& data_kdtree,
				   const std::vector<const vector<int>*>& interest_region_indices,
				   std::vector<std::vector<float> >& results)
{
	results.resize(interest_region_indices.size());

	vector<vector<int>> interest_region_neigbors_neigbors ; 
	interest_region_neigbors_neigbors.resize(interest_region_indices.size()); 

	for (int i = 0 ; i < interest_region_indices.size() ; i ++ )
	{
		for (int j = 0 ; j < interest_region_indices[i]->size() ; j ++ )
		{
			interest_region_neigbors_neigbors[i].push_back(j);
		}
	}

	computeFeatures(data,data_kdtree,interest_region_neigbors_neigbors,results);
}

void FPFH::compute(const PointCloud& data,
				   KdTree& data_kdtree, 
				   const vector<SupervoxelUnit>& supervoxels, 
				   vector<vector<float> >& results)
{
	results.resize(supervoxels.size());

	vector<vector<int>> interest_region_neigbors_neigbors ; 
	interest_region_neigbors_neigbors.resize(supervoxels.size()); 

	for (int i = 0 ; i < supervoxels.size() ; i ++ )
	{
		for (int j = 0 ; j < supervoxels[i].indices.size() ; j ++ )
		{
			interest_region_neigbors_neigbors[i].push_back(supervoxels[i].indices[j]);
		}
	}

	computeFeatures(data,data_kdtree,interest_region_neigbors_neigbors,results);
}
//
//// --------------------------------------------------------------
///* See function definition */
//// --------------------------------------------------------------
//void FPFH::compute(const PointCloud& data,
//				   KdTree& data_kdtree, 
//				   const vector<SupervoxelUnit>& supervoxels, 
//				   vector<vector<float> >& results)
//{
//	cout << "first order" << endl ;
//	//计算一阶领域的特征
//	results.resize(supervoxels.size());
//
//	//used for neighborhood
//	PointCloud::Ptr super_cloud(new PointCloud) ; 
//
//	for (int i = 0 ; i < supervoxels.size() ; i ++ )
//	{
//		PointType point ; 
//		point.x =  supervoxels[i].point.x ;
//		point.y =  supervoxels[i].point.y ;
//		point.z =  supervoxels[i].point.z ;
//		super_cloud->push_back(point);
//	}
//
//	//输出 adjacent neighorhood
//	KdTree super_kdtree ;
//	super_kdtree.setInputCloud(super_cloud);
//
//	
//	vector<vector<int>> interest_region_neigbors_neigbors ; 
//	interest_region_neigbors_neigbors.resize(supervoxels.size()); 
//	for ( int pidx =0 ;pidx < super_cloud->points.size() ; pidx ++ )
//	{
//		vector<int> indices; 
//		vector<float> distances;
//		double radius = 0.3 ; 
//		PointType search_point; 
//		search_point.x = super_cloud->points[pidx].x ; 
//		search_point.y = super_cloud->points[pidx].y ; 
//		search_point.z = super_cloud->points[pidx].z ; 
//
//		int search_count = super_kdtree.radiusSearch(search_point,radius,indices,distances);
//
//		for (int j = 0 ; j < search_count ; j ++ )
//		{
//			for (int k = 0 ; k < supervoxels[indices[j]].indices.size(); k ++ )
//			{
//				interest_region_neigbors_neigbors[pidx].push_back(supervoxels[indices[j]].indices[k]);
//			}
//		}
//
//	}
//
//	computeFeatures(data,data_kdtree,interest_region_neigbors_neigbors,results);
//}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void FPFH::compute(const PointCloud& data, 
				   KdTree& data_kdtree, 
				   const vector< vector<SupervoxelUnit>* > & supervoxels,
				   vector<vector<float> >& results)
{
	results.resize(supervoxels.size());

	vector<vector<int>> interest_region_neigbors_neigbors ; 
	interest_region_neigbors_neigbors.resize(supervoxels.size()); 

	for (int i = 0 ; i < supervoxels.size() ; i ++ )
	{
		for (int j = 0 ; j < supervoxels[i]->size() ; j ++ )
		{
			for (int k = 0 ; k < (*supervoxels[i])[j].indices.size() ; k ++ )
			{
				interest_region_neigbors_neigbors[i].push_back((*supervoxels[i])[j].indices[k]);
			}
		}
	}

	computeFeatures(data,data_kdtree,interest_region_neigbors_neigbors,results);
}




void FPFH::computeFeatures(const PointCloud& data , 
						   KdTree& data_kdtree,
						   vector<vector<int>> interest_region_neigbors_neigbors,
						   std::vector<std::vector<float> >& results)
{
	// compute all the normals 
	Normal_Estimation( data,  data_kdtree ) ;

	//compute centroid for each region

	for (int region_idx = 0 ; 
		region_idx < interest_region_neigbors_neigbors.size() ; region_idx ++ )
	{
		PointCloud neighbor_cloud ;
		pcl::PointCloud<pcl::Normal> neigbor_normals; 
		for (int i = 0 ; i < interest_region_neigbors_neigbors[region_idx].size() ; i ++)
		{
			neighbor_cloud.points.push_back(
				data.points[interest_region_neigbors_neigbors[region_idx][i]]);
			neigbor_normals.push_back(
				_normals[interest_region_neigbors_neigbors[region_idx][i]]);
		}

		if ( neighbor_cloud.points.size() < 5 )
		{
			std::vector<float> fpfh ; 
			results[region_idx] = fpfh ; 
			continue; 
		}

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(neighbor_cloud,centroid);

		//find the point closet to the centroid point, and treat it as the feature point;
		int idx=-1;
		double dis=DBL_MAX;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for(int i=0;i<neighbor_cloud.points.size();i++)
		{
			pcl::PointXYZ tmp;
			tmp.x=neighbor_cloud.points[i].x;
			tmp.y=neighbor_cloud.points[i].y;
			tmp.z=neighbor_cloud.points[i].z;
			cloud->points.push_back(tmp);

			double tmpdis=0.0;
			tmpdis+=(neighbor_cloud.points[i].x-centroid[0])*(neighbor_cloud.points[i].x-centroid[0]);
			tmpdis+=(neighbor_cloud.points[i].y-centroid[1])*(neighbor_cloud.points[i].y-centroid[1]);
			tmpdis+=(neighbor_cloud.points[i].z-centroid[2])*(neighbor_cloud.points[i].z-centroid[2]);
			tmpdis=sqrt(tmpdis);

			if(tmpdis<dis)
			{
				dis=tmpdis;
				idx=i;
			}
		}

		Eigen::Vector4f p0,n0;
		float tmp[3];
		tmp[0]=neighbor_cloud.points[idx].x;
		tmp[1]=neighbor_cloud.points[idx].y;
		tmp[2]=neighbor_cloud.points[idx].z;
		p0(0) = tmp[0];
		p0(1) = tmp[1];
		p0(2) = tmp[2];
		p0(3) = 0.0;

		tmp[0]=neigbor_normals.points[idx].normal_x;
		tmp[1]=neigbor_normals.points[idx].normal_y;
		tmp[2]=neigbor_normals.points[idx].normal_z;
		n0(0) = tmp[0];
		n0(1) = tmp[1];
		n0(2) = tmp[2];
		n0(3) = 0.0;


		// 计算点对PFH
		float fv[4];


		std::vector<std::vector<float>> spfh;

		for(int n=0;n<cloud->points.size();n++)
		{
			if (n==idx)
			{
				continue;
			}

			Eigen::Vector4f p1,n1;

			tmp[0]=cloud->points[n].x;
			tmp[1]=cloud->points[n].y;
			tmp[2]=cloud->points[n].z;
			p1(0) = tmp[0];
			p1(1) = tmp[1];
			p1(2) = tmp[2];
			p1(3) = 0.0;

			tmp[0]=neigbor_normals.points[n].normal_x;
			tmp[1]=neigbor_normals.points[n].normal_y;
			tmp[2]=neigbor_normals.points[n].normal_z;
			n1(0) = tmp[0];
			n1(1) = tmp[1];
			n1(2) = tmp[2];
			n1(3) = 0.0;

			pcl::computePairFeatures(p0,n0,p1,n1,fv[0],fv[1],fv[2],fv[3]);
			std::vector<float> item;
			for(int k=0;k<4;++k)
				item.push_back(fv[k]);
			spfh.push_back(item);
		}

		// 计算特征向量
		float lfv[16];
		for(int m=0;m<16;++m)
			lfv[m] = 0.0;
		for(size_t m=0;m<spfh.size();++m){
			int idx = 0;
			idx += (spfh[m][0]<0)?0:1;
			idx += 2*((spfh[m][1]<0)?0:1);
			idx += 4*((spfh[m][2]<0)?0:1);
			idx += 8*((spfh[m][3]<0.5*_radius_fpfh)?0:1);
			lfv[idx] += 1;
		}

		for(int m=0;m<16;++m)
			lfv[m] /= spfh.size();

		for(int k=0;k<16;++k)
			results[region_idx].push_back(lfv[k]);

	}
	return;

}

void FPFH::Normal_Estimation(const PointCloud& data, KdTree& data_kdtree )
{

	int width=data.points.size();

	//	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);

	//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	//	tree->setInputCloud(pcloud);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for(int i=0;i<width;i++)
	{
		PointType CenterPoint= data.points[i];

		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();

		if (data_kdtree.radiusSearch(CenterPoint, _radius_normal , pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )//找到searchPoint在点集中的最近邻点
		{
			pcl::PointCloud<PointType>::Ptr local_patch(new pcl::PointCloud<PointType>);

			for (int pit=0; pit < pointIdxRadiusSearch.size(); pit++)
				local_patch->points.push_back(data.points[pointIdxRadiusSearch[pit]]);

			local_patch->width=pointIdxRadiusSearch.size();
			local_patch->height=1;
			local_patch->is_dense=true;

			// Placeholder for the 3x3 covariance matrix at each surface patch
			Eigen::Matrix3f covariance_matrix;
			// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
			Eigen::Vector4f xyz_centroid;
			// Estimate the XYZ centroid
			pcl::compute3DCentroid (*local_patch, xyz_centroid);
			// Compute the 3x3 covariance matrix
			pcl::computeCovarianceMatrix (*local_patch, xyz_centroid, covariance_matrix);

			//compute the eigen values

			/*		Eigen::Matrix3f Eigen_vecs;
			Eigen::Vector3f evals;
			pcl::eigen33(covariance_matrix,Eigen_vecs,evals);*/

			EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
			EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
			pcl::eigen33(covariance_matrix,eigen_value,eigen_vector);

			pcl::Normal ne;
			ne.normal_x=eigen_vector[0];
			ne.normal_y=eigen_vector[1];
			ne.normal_z=eigen_vector[2];

			_normals.points.push_back(ne);
		}
	}
}