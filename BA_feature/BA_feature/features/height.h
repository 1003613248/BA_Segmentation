#ifndef __D3D_HEIGHT_H__
#define __D3D_HEIGHT_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
class Height: public Descriptor3D
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
    Height()
    {
      result_size_ = 1;
	  scale_= 1; 
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
};

#endif
