#ifndef __D3D_COLOR_H__
#define __D3D_COLOR_H__

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include "descriptor_3d.h"

#include "nearest.h"

using namespace std;

// --------------------------------------------------------------
//* Position
/*!
 * \brief A Position descriptor simply uses the 3rd coordinate (z) of the
 *        interest point/region, for now.
 */
// --------------------------------------------------------------
class MyColor: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the Position descriptor
     *
     * The computed feature is the z-coordinate of the given interest point
     * or of the centroid from the given interest region
     */
    // --------------------------------------------------------------
    MyColor()
    {
      result_size_ = 3;
	  scale_= 1; 
    }

	void setParameters(double radius)
	{
		radius_ = radius  ;
	}

    // TODO use sensor location so height is relative and dont assume flat ground
    //void useSensorLocation();

    // TODO: use map information such as distance from known walls
    //void useMapInformation(void* mapData);

    // --------------------------------------------------------------
    /*!
     * \brief Extract the z-coordinate of the interest points
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
     * \brief Extract the z-coordinate of the interest regions' centroids
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


public:
	double scale_;
	double radius_ ;
};

#endif
