#include <direct.h>
#include <iostream>
#include <stdio.h>
#include <string>
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
#include "supervoxel/Supervoxel.h"
#include "features/all_descriptors.h"
#include "utils/misc.h"

using namespace std;

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
		string folderPath = dir + filename + "/Data";
		if (0 != access(folderPath.c_str(), 0))
		{
			mkdir(folderPath.c_str());
		}
		for (int i = 0; i < n; ++i)
		{
			pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh_omp;
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
			string str = std::to_string(i + 1);
			const std::string dirt = dir + filename + "/" + str + "-region" + ".xyz";
			cl::Array<cl::RPoint3D> points;
			cl::Array<cl::RGB32Color> colors;
			cout << "Reading points from " << dirt << "..." << endl;
			if (!cl::geometry::io::ReadXYZPoints(dirt.c_str(), &points, &colors)) {
				cout << "Please check if " << dirt << " is exist." << endl;
				return 0;
			}

			/*     Supervoxel Segmentation     */

			cl::Array<cl::RGB32Color> colors_p = colors;
			int n_points = points.size();
			cout << n_points << "Points are imported." << endl;
			//cout << "Building KD tree..." << endl;
			cl::KDTree<cl::RPoint3D> kdtree;
			kdtree.SwapPoints(&points);
			const int k_neighbors = 15;
			assert(k_neighbors < n_points);
			//cout << "Compute the k-nearest neighbors for each point, and estimate the normal vectors..." << endl;
			cl::Array<cl::RVector3D> snormals(n_points);
			cl::Array<cl::Array<int> > neighbors(n_points);
			cl::Array<cl::RPoint3D> neighbor_points(k_neighbors);
			for (int i = 0; i < n_points; ++i)
			{
				kdtree.FindKNearestNeighbors(kdtree.points()[i], k_neighbors, &neighbors[i]);
				for (int k = 0; k < k_neighbors; ++k)
				{
					neighbor_points[k] = kdtree.points()[neighbors[i][k]];
				}
				cl::geometry::point_cloud::PCAEstimateNormal(neighbor_points.begin(), neighbor_points.end(), &snormals[i]);
			}
			kdtree.SwapPoints(&points);
			auto t3 = chrono::steady_clock::now();
			cl::Array<PointWithNormal> oriented_points(n_points);
			for (int i = 0; i < n_points; ++i)
			{
				oriented_points[i].x = points[i].x;
				oriented_points[i].y = points[i].y;
				oriented_points[i].z = points[i].z;
				oriented_points[i].normal = snormals[i];
			}
			const double resolution = 0.15;
			VCCSMetric metric(resolution);
			cl::Array<int> labels, supervoxels;
			cl::geometry::point_cloud::SupervoxelSegmentation(oriented_points, neighbors, resolution, metric, &supervoxels, &labels);
			int n_supervoxels = supervoxels.size();
			std::mt19937 random;
			cl::Array<cl::RGB32Color> supervoxel_colors(n_supervoxels);
			for (int i = 0; i < n_supervoxels; ++i)
			{
				supervoxel_colors[i] = cl::RGB32Color(random());
			}
			for (int i = 0; i < n_points; ++i)
			{
				colors[i] = supervoxel_colors[labels[i]];
			}
			auto t4 = chrono::steady_clock::now();
			auto dt1 = chrono::duration_cast<chrono::duration<double>>(t4 - t3).count();
			int **supervindices = new int*[n_supervoxels];
			int *pnumofsuperv = new int[n_supervoxels];
			if (cl::geometry::io::WriteXYZPoints((dir + filename + "/Data/" + str + "-supervoxel" + ".xyz").c_str(), points, colors))
			{
			}
			kdtree.clear();
			ofstream repoint((dir + filename + "/Data/" + str + "-r_target" + ".xyz").c_str());
			for (int i = 0; i < n_supervoxels; ++i)
			{
				supervindices[i] = new int[1000];
				pnumofsuperv[i] = 0;
				repoint << points[supervoxels[i]].x << " " << points[supervoxels[i]].y << " " << points[supervoxels[i]].z << " " << (int)colors_p[supervoxels[i]].red() << " " << (int)colors_p[supervoxels[i]].green() << " " << (int)colors_p[supervoxels[i]].blue() << endl;
			}
			for (int i = 0; i < n_points; ++i)
			{
				supervindices[labels[i]][pnumofsuperv[labels[i]]] = i;
				pnumofsuperv[labels[i]]++;
			}
			ofstream superindices((dir + filename + "/Data/" + str + "-indices_target" + ".txt").c_str());
			for (int i = 0; i < n_supervoxels; i++)
			{
				superindices << pnumofsuperv[i] << " ";
				for (int j = 0; j < pnumofsuperv[i]; ++j)
				{
					superindices << supervindices[i][j];
					if (j == (pnumofsuperv[i] - 1)) superindices << endl;
					else superindices << " ";
				}
			}
			for (int i = 0; i < n_supervoxels; ++i)
			{
				delete[] supervindices[i];
			}
			delete[] supervindices;
			delete[] pnumofsuperv;

			/*      Feature descriptors      */
			/*     FPFH and Orientation Descriptors     */

			pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
			model->width = n_supervoxels;
			model->height = 1;
			model->is_dense = false;
			model->points.resize(model->width*model->height);
			for (int i = 0; i < model->points.size(); ++i)
			{
				model->points[i].x = points[supervoxels[i]].x;
				model->points[i].y = points[supervoxels[i]].y;
				model->points[i].z = points[supervoxels[i]].z;
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr model1(new pcl::PointCloud<pcl::PointXYZ>); 
			model1->width = points.size();
			model1->height = 1;
			model1->points.resize(model1->width*model1->height);
			for (int i = 0; i < points.size(); i++)
			{
				model1->points[i].x = points[i].x;
				model1->points[i].y = points[i].y;
				model1->points[i].z = points[i].z;
			}
			PointCloud::Ptr pcloud(new PointCloud);
			for (int ii = 0; ii < points.size(); ii++)
			{
				PointType point;
				point.x = points[ii].x;
				point.y = points[ii].y;
				point.z = points[ii].z;
				pcloud->points.push_back(point);
			}
			KdTree pt_cloud_kdtree;
			pt_cloud_kdtree.setInputCloud(pcloud);
			pcl::KdTreeFLANN<pcl::PointXYZ>kdtree2;
			kdtree2.setInputCloud(model1);
			int num_k = 50;
			pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
			pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			ne.setInputCloud(model);
			ne.setSearchMethod(tree);
			ne.setRadiusSearch(0.3);
			ne.compute(*normals);
			fpfh_omp.setInputCloud(model);
			fpfh_omp.setInputNormals(normals);
			fpfh_omp.setSearchMethod(tree);
			fpfh_omp.setNumberOfThreads(8);
			fpfh_omp.setRadiusSearch(0.3);
			fpfh_omp.compute(*fpfhs);
			pcl::FPFHSignature33 descriptor;
			double b = 100.0;
			ofstream FPFHdata((dir + filename + "/Data/" + str + "-FPFH" + ".txt").c_str());
			for (int i = 0; i < n_supervoxels; ++i)
			{
				descriptor = fpfhs->points[i];
				for (int j = 0; j < 33; j++)
				{
					FPFHdata << descriptor.histogram[j] << " ";
				}
				Orientation* orientation = new Orientation();
				orientation->setSpectralRadius(0.35);
				orientation->useNormalOrientation(0.0, 0.0, 1.0);
				orientation->useTangentOrientation(0.0, 0.0, 1.0);
				vector<const PointType*> interest_p;
				vector<int> indices(num_k);
				vector<float> distin(num_k);
				kdtree2.nearestKSearch(model1->points[supervoxels[i]], num_k, indices, distin);
				for (int ii = 0; ii < num_k; ii++)
				{
					PointType* ipoint;
					PointType point;
					point.x = points[indices[ii]].x;
					point.y = points[indices[ii]].y;
					point.z = points[indices[ii]].z;
					ipoint = &point;
					interest_p.push_back(ipoint);
				}
				vector<float>().swap(distin);
				vector<int>().swap(indices);
				vector<vector<float>> descriptor_results;
				orientation->compute(*pcloud, pt_cloud_kdtree, interest_p, descriptor_results);
				FPFHdata << b*descriptor_results[0][0] << " " << b*descriptor_results[0][1] << endl;

			}
			FPFHdata.close();
			repoint.close();
			superindices.close();
			cout << ".";
			fpfhs->clear();
			normals->clear();
			model->clear();
			model1->clear();
			kdtree2.~KdTreeFLANN();
			pt_cloud_kdtree.~KdTree();
		}
		cout << "Complete." << endl;
	}
	return 0;
}