#ifndef __D3D_DESCRIPTORS_3D_H__
#define __D3D_DESCRIPTORS_3D_H__

#include <vector>
#include <set>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include "../utils/types.h"
#include "../supervoxel/Supervoxel.h"

using namespace std;

// --------------------------------------------------------------
//* Descriptor3D
/*!
 * \brief An abstract class representing a descriptor that can
 *        compute feature values from 3-D data
 */
// --------------------------------------------------------------

class Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates a descriptor with 0 feature values
     */
    // --------------------------------------------------------------
    Descriptor3D() :
      result_size_(0)
    {
    }

    virtual ~Descriptor3D() = 0;

    // ===================================================================
    /*! \name Virtual methods */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each specified interest point
     *
     * See the inherited class constructor for the type of features computed
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_pts List of interest points to compute features for
     * \param results Vector to hold computed vector of features for each interest point.
     *                If the features could not be computed for an interest point i, then
     *                results[i].size() = 0
     */
    // --------------------------------------------------------------
    virtual void compute(const PointCloud & data,
                         KdTree& data_kdtree,
                         const vector<const PointType*>& interest_pts,
                         vector<vector<float> >& results) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each interest region of points
     *
     * See the inherited class constructor for the type of features computed
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_region_indices List of groups of indices into data that represent an interest region
     * \param results Vector to hold computed vector of features for each interest point.
     *                If the features could not be computed for an interest point i, then
     *                results[i].size() = 0
     */
    // --------------------------------------------------------------
    virtual void compute(const PointCloud& data,
                         KdTree& data_kdtree,
                         const vector<const vector<int>*>& interest_region_indices,
                         vector<vector<float> >& results) = 0;


	//node for supervoxel
    virtual void compute(const PointCloud& data,
                         KdTree& data_kdtree,
						 const vector<SupervoxelUnit>& supervoxels,
                         vector<vector<float> >& results) = 0;


	//clique for supervoxels 
	virtual void compute(const PointCloud& data,
                         KdTree& data_kdtree,
						 const vector< vector<SupervoxelUnit>* > & supervoxels,
                         vector<vector<float> >& results) = 0;

    //@}

    // --------------------------------------------------------------
    /*!
     * \brief Returns the number of feature values this descriptor computes on success
     *
     * \return the number of feature values this descriptor computes on success
     */
    // --------------------------------------------------------------
    inline unsigned int getResultSize() const
    {
      return result_size_;
    }

    // ===================================================================
    /*! \name Utility methods */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief Utility function to compute multiple descriptor feature around
     *        interest points and concatenate the results into a single vector
     *
     * \param data See Descriptor3D::compute
     * \param data_kdtree See Descriptor3D::compute
     * \param interest_pts See Descriptor3D::compute
     * \param descriptors_3d List of various feature descriptors to compute on each interest point
     * \param concatenated_features List containing the concatenated features from the descriptors if
     *                              they were ALL successful for the interest point.  If one descriptor
     *                              failed for interest point i, then concatenated_features[i] == NULL
     * \param failed_indices The set of indices in concatenated_features that are NULL
     *
     * \warning It is up to the user to free the concatenated features
     *
     * \return The total number of concatenated feature values
     */
    // --------------------------------------------------------------
    static unsigned int computeAndConcatFeatures(const PointCloud& data,
                                                 KdTree& data_kdtree,
                                                 const vector<const PointType*>& interest_pts,
                                                 vector<Descriptor3D*>& descriptors_3d,
                                                 vector<float*>& concatenated_features,
                                                 set<unsigned int>& failed_indices);


	static unsigned int computeAndConcatFeatures(const PointCloud& data,
												 KdTree& data_kdtree,
												 const vector<SupervoxelUnit>& supervoxels,
												 vector<Descriptor3D*>& descriptors_3d,
		                                         vector<float*>& concatenated_features,
												 set<unsigned int>& failed_indices);

    // --------------------------------------------------------------
    /*!
     * \brief Utility function to compute multiple descriptor feature around
     *        interest regions and concatenate the results into a single vector
     *
     * \param data See Descriptor3D::compute
     * \param data_kdtree See Descriptor3D::compute
     * \param interest_region_indices See Descriptor3D::compute
     * \param descriptors_3d List of various feature descriptors to compute on each interest point
     * \param concatenated_features List containing the concatenated features from the descriptors if
     *                              they were ALL successful for the interest region.  If one descriptor
     *                              failed for interest region i, then concatenated_features[i] == NULL
     * \param failed_indices The set of indices in concatenated_features that are NULL
     *
     * \warning It is up to the user to free the concatenated features
     *
     * \return The total number of concatenated feature values
     */
    // --------------------------------------------------------------
    static unsigned int
    computeAndConcatFeatures(const PointCloud& data,
                             KdTree& data_kdtree,
                             const vector< vector<SupervoxelUnit>* > & supervoxels,
                             vector<Descriptor3D*>& descriptors_3d,
                             vector<float*>& concatenated_features,
                             set<unsigned int>& failed_indices);


	static unsigned int
		computeAndConcatFeatures(const PointCloud& data,
		KdTree& data_kdtree,
		const vector<const vector<int>*>& interest_region_indices,
		vector<Descriptor3D*>& descriptors_3d,
		vector<float*>& concatenated_features,
		set<unsigned int>& failed_indices);
    //@}

  protected:
    /*! \brief The number of feature values the descriptors computes on success */
    unsigned int result_size_;

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Concatenates the resulting feature descriptor values
     *
     * \param all_descriptor_results The results for each descriptor
     * \param nbr_samples The number of interest points/regions
     * \param nbr_concatenated_vals The total length of all concatenated features from
     *                              each descriptor
     * \param concatenated_features The concatenated features.  NULL indicates could not
     *                              successfully compute descriptor for sample
     * \param failed_indices The indices in concatenated_features that are NULL
     */
    // --------------------------------------------------------------
    static void concatenateFeatures(const vector<vector<vector<float> > >& all_descriptor_results,
                                    const unsigned int nbr_samples,
                                    const unsigned int nbr_concatenated_vals,
                                    vector<float*>& concatenated_features,
                                    set<unsigned int>& failed_indices);
};

#endif
