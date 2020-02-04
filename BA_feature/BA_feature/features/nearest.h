#ifndef _CLOUD_GEOMETRY_NEAREST_H_
#define _CLOUD_GEOMETRY_NEAREST_H_

//#define M_PI 3.141592653

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

#include "utils/types.h"

namespace cloud_geometry
{

	namespace nearest
	{
		inline void
			computeCentroid (const PointCloud &points, const std::vector<int> &indices, PointType &centroid)
		{
			centroid.x = centroid.y = centroid.z = 0;
			// For each point in the cloud
			for (unsigned int i = 0; i < indices.size (); i++)
			{
				centroid.x += points.at (indices.at (i)).x;
				centroid.y += points.at (indices.at (i)).y;
				centroid.z += points.at (indices.at (i)).z;
			}

			centroid.x /= indices.size ();
			centroid.y /= indices.size ();
			centroid.z /= indices.size ();
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
		* The result is returned as a Eigen::Matrix3d.
		* \note The (x-y-z) centroid is also returned as a Point3D message.
		* \param points the input point cloud
		* \param indices the point cloud indices that need to be used
		* \param covariance_matrix the 3x3 covariance matrix
		* \param centroid the computed centroid
		*/
		inline void
			computeCovarianceMatrix (const PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix, PointType &centroid)
		{
			computeCentroid (points, indices, centroid);

			// Initialize to 0
			covariance_matrix = Eigen::Matrix3d::Zero ();

			for (unsigned int j = 0; j < indices.size (); j++)
			{
				covariance_matrix (0, 0) += (points[indices.at (j)].x - centroid.x) * (points[indices.at (j)].x - centroid.x);
				covariance_matrix (0, 1) += (points[indices.at (j)].x - centroid.x) * (points[indices.at (j)].y - centroid.y);
				covariance_matrix (0, 2) += (points[indices.at (j)].x - centroid.x) * (points[indices.at (j)].z - centroid.z);

				covariance_matrix (1, 0) += (points[indices.at (j)].y - centroid.y) * (points[indices.at (j)].x - centroid.x);
				covariance_matrix (1, 1) += (points[indices.at (j)].y - centroid.y) * (points[indices.at (j)].y - centroid.y);
				covariance_matrix (1, 2) += (points[indices.at (j)].y - centroid.y) * (points[indices.at (j)].z - centroid.z);

				covariance_matrix (2, 0) += (points[indices.at (j)].z - centroid.z) * (points[indices.at (j)].x - centroid.x);
				covariance_matrix (2, 1) += (points[indices.at (j)].z - centroid.z) * (points[indices.at (j)].y - centroid.y);
				covariance_matrix (2, 2) += (points[indices.at (j)].z - centroid.z) * (points[indices.at (j)].z - centroid.z);
			}
		}


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief Compute the eigenvalues and eigenvectors of a given surface patch from its normalized covariance matrix
		* \param points the input point cloud
		* \param indices the point cloud indices that need to be used
		* \param eigen_vectors the resultant eigenvectors
		* \param eigen_values the resultant eigenvalues
		* \param centroid the centroid of the points
		*/
		inline void
			computePatchEigenNormalized (const PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values, PointType& centroid)
		{

			// Compute the 3x3 covariance matrix
			Eigen::Matrix3d covariance_matrix;
			computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

			// // Normalize the matrix by 1/N
			for (unsigned int i = 0 ; i < 3 ; i++)
			{
				for (unsigned int j = 0 ; j < 3 ; j++)
				{
					covariance_matrix(i, j) /= static_cast<double> (indices.size ());
				}
			}


			// // Extract the eigenvalues and eigenvectors
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
			eigen_values = ei_symm.eigenvalues ();
			eigen_vectors = ei_symm.eigenvectors ();
		}
	}
}


#endif