#ifndef __PCC_H__
#define __PCC_H__
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

#include <set>
#include <map>
#include <vector>
#include <list>

#include <pcl/kdtree/kdtree_flann.h>

#include "supervoxel/Supervoxel.h"

#include "utils/types.h"

namespace point_cloud_clustering
{
  class PointCloudClustering
  {
    public:
      static int computeClusterCentroids(const PointCloud& pt_cloud, const std::map<unsigned int,
          std::vector<int> >& clusters, std::map<unsigned int, std::vector<float> >& cluster_centroids);

      PointCloudClustering();

      virtual ~PointCloudClustering() = 0;

      inline void setStartingClusterLabel(unsigned int starting_label)
      {
        starting_label_ = starting_label;
      }

	  int cluster(const vector<SupervoxelUnit>& units,
		  std::map<unsigned int, std::vector<int> >& created_clusters);

      virtual int cluster(const  vector<SupervoxelUnit>& units,
                          KdTree& pt_cloud_kdtree,
                          const std::set<unsigned int>& indices_to_cluster,
                          std::map<unsigned int, std::vector<int> >& created_clusters) = 0;

    protected:
      unsigned int findRadiusNeighbors(KdTree& pt_cloud_kdtree,
                                       unsigned int index,
                                       double radius,
                                       const std::set<unsigned int>& indices_to_cluster,
                                       std::list<unsigned int>& neighbor_indices);

      bool parameters_defined_;
      unsigned int starting_label_;
  };
}

#endif
