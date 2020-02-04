#include <direct.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <math.h>
#include "codelibrary/base/log.h"
#include "codelibrary/geometry/io/xyz_io.h"
#include "codelibrary/geometry/point_cloud/pca_estimate_normals.h"
#include "codelibrary/geometry/point_cloud/supervoxel_segmentation.h"
#include "codelibrary/geometry/util/distance_3d.h"
#include "codelibrary/util/tree/kd_tree.h"
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/pcl_visualizer.h>   
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
struct PointWithNormal : cl::RPoint3D {
	PointWithNormal() {}

	cl::RVector3D normal;
};

class VCCSMetric {
public:
	explicit VCCSMetric(double resolution)
		: resolution_(resolution) {}

	double operator() (const PointWithNormal& p1,
		const PointWithNormal& p2) const {
		return 1.0 - std::fabs(p1.normal * p2.normal) +
			cl::geometry::Distance(p1, p2) / resolution_ * 0.4;
	}
private:
	double resolution_;
};

boost::mutex cloud_mutex;

double standard(vector<float> &data)
{
	double sum = 0.0;
	double average;
	double variance = 0.0;
	int n = data.size();
	for (int i = 1; i < n; ++i)
	{
		sum = sum + data[i];
	}
	average = sum / n;
	for (int i = 1; i < n; ++i)
	{
		variance = variance + pow((average - data[i]), 2);
	}
	variance = variance / n;
	return sqrt(variance);
}

int main()
{
	int c = 0;
	ifstream list("..\\..\\Data/List1.txt");
	string dir = "..\\..\\Data/";
	string filenameandnumber;
	cl::RGB32Color red = cl::RGB32Color(255, 0, 0);
	cl::RGB32Color white = cl::RGB32Color(255, 255, 255);
	cl::RGB32Color back = cl::RGB32Color(0, 255, 255);
	while (getline(list, filenameandnumber))
	{
		++c;
		cout << c << ": ";
		string filename;
		int n;
		int pointnum, cloudnum;
		stringstream ss;
		ss << filenameandnumber;
		ss >> filename;
		ss >> n;
		string folderPath = dir + filename + "/Edge_datasets";
		if (0 != access(folderPath.c_str(), 0))
		{
			mkdir(folderPath.c_str());
		}
		string folderPath1 = dir + filename + "/Supervoxeltest";
		if (0 != access(folderPath1.c_str(), 0))
		{
			mkdir(folderPath1.c_str());
		}
		for (int i = 0; i < n; ++i)
		{
			cl::Array<cl::RPoint3D> points;
			cl::Array<cl::RGB32Color> colors;
			string str = std::to_string(i + 1);
			//cout << "Reading points from " << dirt << "..." << endl;
			const std::string dirt = dir + filename + "/" + str + "-region" + ".xyz";
			if (!cl::geometry::io::ReadXYZPoints(dirt.c_str(), &points, &colors)) {
				cout << "Please check if " << dirt << " is exist." << endl;
				return 0;
			}
			cl::Array<cl::RGB32Color> colors_c = colors;
			pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
			model->width = points.size();
			model->height = 1;
			model->points.resize(model->width*model->height);
			for (int i = 0; i < points.size(); i++)
			{
				model->points[i].x = points[i].x;
				model->points[i].y = points[i].y;
				model->points[i].z = points[i].z;
			}
			int M;
			double R;
			int T;
			T = 100;
			M = 51;
			R = 0.125;

			/*     Boundary Generation    */

			/*   Compute the k-nearest neighbors   */
			vector<vector<int>> neighborsIndexes;
			vector<vector<float>> pointNKNSquaredDistance;
			vector<int> numofNeighbors;
			pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
			kdtree.setInputCloud(model);
			for (int i = 0; i < points.size(); i++)
			{
				vector<int> kn_indexes;
				vector<float> kn_distance;
				vector<int> rn_indexes;
				vector<float> rn_distance;
				kdtree.nearestKSearch(model->points[i], M, kn_indexes, kn_distance);
				kdtree.radiusSearch(model->points[i], R, rn_indexes, rn_distance);
				if (kn_indexes.size() > rn_indexes.size())
				{
					neighborsIndexes.push_back(kn_indexes);
					pointNKNSquaredDistance.push_back(kn_distance);
					numofNeighbors.push_back(kn_indexes.size());
				}
				else
				{
					neighborsIndexes.push_back(rn_indexes);
					pointNKNSquaredDistance.push_back(rn_distance);
					numofNeighbors.push_back(rn_indexes.size());
				}
				vector<float>().swap(kn_distance);
				vector<int>().swap(kn_indexes);
				vector<float>().swap(rn_distance);
				vector<int>().swap(rn_indexes);
			}
			vector<double> I_n;
			for (int i = 0; i < points.size(); i++)
			{
				double px, py, pz;
				double P_ix, P_iy, P_iz;
				double I = 0;
				px = py = pz = 0;
				for (int j = 0; j < numofNeighbors[i]; j++)
				{
					px = px + points[neighborsIndexes[i][j]].x;
					py = py + points[neighborsIndexes[i][j]].y;
					pz = pz + points[neighborsIndexes[i][j]].z;
				}
				P_ix = px / numofNeighbors[i];
				P_iy = py / numofNeighbors[i];
				P_iz = pz / numofNeighbors[i];
				I = (sqrt(pow((points[i].x - P_ix), 2) + pow((points[i].y - P_iy), 2) + pow((points[i].z - P_iz), 2))) / R;
				I_n.push_back(I);
			}
			for (int i = 0; i < points.size(); i++)
			{
				colors[i] = white;

				if (pow(I_n[i] * R, 2) / 1.25 > pointNKNSquaredDistance[i][1])
				{
					colors[i] = red;
				}
				
			}

			if (cl::geometry::io::WriteXYZPoints((dir + filename + "/Edge_datasets/" + str + "-edge" + ".xyz").c_str(), points, colors))
			{
			}

			/*      Meaningless Boundary Removal      */

			int n_points = points.size();
			cl::KDTree<cl::RPoint3D> kdtree1;
			kdtree1.SwapPoints(&points);
			const int k_neighbors = 15;
			assert(k_neighbors < n_points);
			cl::Array<cl::RVector3D> snormals(n_points);
			cl::Array<cl::Array<int> > neighbors(n_points);
			cl::Array<cl::RPoint3D> neighbor_points(k_neighbors);
			for (int i = 0; i < n_points; ++i)
			{
				kdtree1.FindKNearestNeighbors(kdtree1.points()[i], k_neighbors, &neighbors[i]);
				for (int k = 0; k < k_neighbors; ++k)
				{
					neighbor_points[k] = kdtree1.points()[neighbors[i][k]];
				}
				cl::geometry::point_cloud::PCAEstimateNormal(neighbor_points.begin(), neighbor_points.end(), &snormals[i]);
			}
			kdtree1.SwapPoints(&points);
			//cout << "Start supervoxel segmentation..." << endl;
			cl::Array<PointWithNormal> oriented_points(n_points);
			for (int i = 0; i < n_points; ++i)
			{
				oriented_points[i].x = points[i].x;
				oriented_points[i].y = points[i].y;
				oriented_points[i].z = points[i].z;
				oriented_points[i].normal = snormals[i];
			}
			const double resolution = 0.8;//1.75
			VCCSMetric metric(resolution);
			cl::Array<int> labels, supervoxels;
			cl::geometry::point_cloud::SupervoxelSegmentation(oriented_points, neighbors, resolution, metric, &supervoxels, &labels);
			int n_supervoxels = supervoxels.size();
			//cout << n_supervoxels << "Supervoxels computed." << endl;
			std::mt19937 random;
			cl::Array<cl::RGB32Color> supervoxel_colors(n_supervoxels);
			for (int i = 0; i < n_supervoxels; ++i)
			{
				supervoxel_colors[i] = cl::RGB32Color(random());
			}
			for (int i = 0; i < n_points; ++i)
			{
				colors_c[i] = supervoxel_colors[labels[i]];
			}

			for (int i = 0; i < points.size(); i++)
			{
				int flag = 0;
				colors_c[i] = white;
				for (int j = 1; j < 5; ++j)
				{
					if (labels[neighborsIndexes[i][j]] != labels[i])
					{
						colors_c[i] = red;
						break;
					}
				}
				if (colors[i] == red)
				{
					for (int j = 1; j < 5; ++j)
					{
						if (labels[neighborsIndexes[i][j]] != labels[i])
						{
							flag = 1;
						}
					}
					if (flag == 0)
					{
						colors[i] = white;
					}
				}
			}

			const double resolution1 = 0.65;//1.5
			VCCSMetric metric1(resolution1);
			cl::Array<int> labels1, supervoxels1;
			cl::geometry::point_cloud::SupervoxelSegmentation(oriented_points, neighbors, resolution1, metric1, &supervoxels1, &labels1);
			n_supervoxels = supervoxels1.size();
			//cout << n_supervoxels << "Supervoxels computed." << endl;
			cl::Array<cl::RGB32Color> supervoxel_colors1(n_supervoxels);
			for (int i = 0; i < n_supervoxels; ++i)
			{
				supervoxel_colors1[i] = cl::RGB32Color(random());
			}
			for (int i = 0; i < n_points; ++i)
			{
				colors_c[i] = supervoxel_colors1[labels1[i]];
			}
			//cout << "Time cost of Supervoxels: " << dt1 << endl;


			kdtree1.clear();
			for (int i = 0; i < points.size(); i++)
			{
				int flag = 0;
				colors_c[i] = white;
				for (int j = 1; j < 5; ++j)
				{
					if (labels1[neighborsIndexes[i][j]] != labels1[i])
					{
						colors_c[i] = red;
						break;
					}
				}
				if (colors[i] == red)
				{
					for (int j = 1; j < 5; ++j)
					{
						if (labels1[neighborsIndexes[i][j]] != labels1[i])
						{
							flag = 1;
						}
					}
					if (flag == 0)
					{
						colors[i] = white;
					}
				}
			}


			const double resolution2 = 0.5;//1.0
			VCCSMetric metric2(resolution1);
			cl::Array<int> labels2, supervoxels2;
			cl::geometry::point_cloud::SupervoxelSegmentation(oriented_points, neighbors, resolution2, metric2, &supervoxels2, &labels2);
			n_supervoxels = supervoxels2.size();
			//cout << n_supervoxels << "Supervoxels computed." << endl;
			cl::Array<cl::RGB32Color> supervoxel_colors2(n_supervoxels);
			for (int i = 0; i < n_supervoxels; ++i)
			{
				supervoxel_colors2[i] = cl::RGB32Color(random());
			}
			for (int i = 0; i < n_points; ++i)
			{
				colors_c[i] = supervoxel_colors2[labels1[i]];
			}
			//cout << "Time cost of Supervoxels: " << dt1 << endl;


			for (int i = 0; i < points.size(); i++)
			{
				int flag = 0;
				colors_c[i] = white;
				for (int j = 1; j < 5; ++j)
				{
					if (labels2[neighborsIndexes[i][j]] != labels2[i])
					{
						colors_c[i] = red;
						break;
					}
				}
				if (colors[i] == red)
				{
					for (int j = 1; j < 5; ++j)
					{
						if (labels2[neighborsIndexes[i][j]] != labels2[i])
						{
							flag = 1;
						}
					}
					if (flag == 0)
					{
						colors[i] = white;
					}
				}
			}
			cl::Array<cl::RGB32Color> colors_s = colors;
			for (int i = 0; i < points.size(); i++)
			{
				int flag = 0;
				if (colors[i] == red)
				{
					for (int j = 1; j < 5; ++j)
					{
						colors_s[neighborsIndexes[i][j]] = red;
					}
				}
			}
			if (cl::geometry::io::WriteXYZPoints((dir + filename + "/Data/" + str + "-edge" + ".xyz").c_str(), points, colors_s))
			{
				//cout << "The points are written into edge.xyz" << endl;
			}
			for (int i = 0; i < points.size(); i++)
			{
				vector<float>().swap(pointNKNSquaredDistance[i]);
				vector<int>().swap(neighborsIndexes[i]);
			}
			vector<vector<float>>().swap(pointNKNSquaredDistance);
			vector<vector<int>>().swap(neighborsIndexes);
		}
		cout << "Complete." << endl;
	}
	return 0;
}