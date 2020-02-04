#ifndef __D3D_FPFH_H__
#define __D3D_FPFH_H__


#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <pcl/features/normal_3d.h>

#include <pcl/features/pfh.h>

#include "descriptor_3d.h"


class FPFH: public Descriptor3D
{
public:

	FPFH(){ result_size_ = 16 ;}
	// --------------------------------------------------------------
	/*!
	* \brief Computes the saliency features that describe the local
	*        shape around the interest points
	*
	* \warning setSpectralRadius() or useSpectralInformation() must be called first
	*
	* \see Descriptor3D::compute
	*/
	// --------------------------------------------------------------
	virtual void compute(const PointCloud& data,
		KdTree& data_kdtree,
		const std::vector<const PointType*>& interest_pts,
		std::vector<std::vector<float> >& results);

	// --------------------------------------------------------------
	/*!
	* \brief Computes the saliency features that describe the local
	*        shape around/in the interest regions
	*
	* \warning setSpectralRadius() or useSpectralInformation() must be called first
	*
	* \see Descriptor3D::compute
	*/
	// --------------------------------------------------------------
	virtual void compute(const PointCloud& data,
		KdTree& data_kdtree,
		const std::vector<const vector<int>*>& interest_region_indices,
		std::vector<std::vector<float> >& results);

	virtual void compute(	const PointCloud& data,
		KdTree& data_kdtree,
		const vector<SupervoxelUnit>& supervoxels,
		vector<vector<float> >& results) ;

	virtual void compute(	const PointCloud& data,
		KdTree& data_kdtree,
		const vector< vector<SupervoxelUnit>* > & supervoxels,
		vector<vector<float> >& results) ;


	void setParameters(double _r_fpfh, double _r_normal)
	{
		_radius_fpfh = _r_fpfh ; 
		_radius_normal = _r_normal ;
	}

protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the spectral features
     *
     * \param results Container to hold computed features for each interest sample
     */
    // --------------------------------------------------------------
	void computeFeatures(const PointCloud& data , 
		KdTree& data_kdtree,
		vector<vector<int>> interest_region_indices_neigbors,
		std::vector<std::vector<float> >& results);

	void Normal_Estimation(const PointCloud& data,
		KdTree& data_kdtree );

public:

	pcl::PointCloud<pcl::Normal> _normals ;
	double _radius_fpfh ;
	double _radius_normal ;

};


#endif