#include <iostream>
#include <stdio.h>
#include <string>
#include <chrono>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <assert.h>
#include <stdlib.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>             
#include <pcl/features/fpfh_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "codelibrary/base/log.h"
#include "codelibrary/geometry/io/xyz_io.h"
#include "codelibrary/geometry/point_cloud/pca_estimate_normals.h"
#include "codelibrary/geometry/point_cloud/supervoxel_segmentation.h"
#include "codelibrary/geometry/util/distance_3d.h"
#include "codelibrary/util/tree/kd_tree.h"
#include "GCoptimization.h"
#include "KMeans.h"
#include "GMM.h"
#define d 35
using namespace std;
double *datas = new double[100000 * d];
double *p_edgep = new double[100000];
double smoothweight = 0.0;
double edgeweight = 0.0;
double smoothFn(int p1, int p2, int l1, int l2)
{
	double scost = 0.0;
	double a, b, c;
	if (l1 != l2)
	{
		for (int i = 0; i < d; i++)
		{
			a = datas[p1 * d + i];
			b = datas[p2 * d + i];
			c = a + b;
			if (c < 0.00000001)
			{
				c = 0.00000001;
			}
			scost += pow((a - b), 2) / c;
		}

		scost = exp(-(0.001 * scost));
		scost = scost + edgeweight / smoothweight * exp(-1.0 * (p_edgep[p1] + p_edgep[p2]));
	}
	return scost;
}
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
	smoothweight = 100.0;
	edgeweight = 300.0;
	cout << endl;
	cout << "Sweight=" << smoothweight << " " << "Eweight=" << edgeweight << endl;
	int c = 0;
	cl::RGB32Color red = cl::RGB32Color(255, 0, 0);
	cl::RGB32Color white = cl::RGB32Color(255, 255, 255);
	cl::Array<cl::RGB32Color> kcolors;
	kcolors.resize(8);
	kcolors[0] = cl::RGB32Color(255, 0, 0);
	kcolors[1] = cl::RGB32Color(255, 255, 0);
	kcolors[2] = cl::RGB32Color(0, 255, 0);
	kcolors[3] = cl::RGB32Color(255, 0, 255);
	kcolors[4] = cl::RGB32Color(0, 0, 255);
	kcolors[5] = cl::RGB32Color(0, 255, 255);
	kcolors[6] = cl::RGB32Color(125, 125, 125);
	kcolors[7] = cl::RGB32Color(255, 255, 255);
	ifstream list("..\\..\\Data/List1.txt");
	string dir = "..\\..\\Data/";
	string filenameandnumber;
	double s_F1, s_Pre, s_Re;
	s_F1 = s_Pre = s_Re = 0.0;
	int n_t = 0;
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
		n_t = n_t + n;
		for (int i = 0; i < n; ++i)
		{
			//cout << i + 1 << " : " << endl;
			string str = std::to_string(i + 1);
			int num_neigh = 5;
			int iter = 10;
			std::string otfilename;
			string it[10] = { "-1.xyz","-2.xyz","-3.xyz" ,"-4.xyz" ,"-5.xyz" ,"-6.xyz" ,"-7.xyz" ,"-8.xyz" ,"-9.xyz" ,"-10.xyz" };
			cl::Array<cl::RPoint3D> points;
			cl::Array<cl::RGB32Color> colors;
			cl::Array<cl::RPoint3D> allpoints;
			cl::Array<cl::RGB32Color> allcolors;
			cl::Array<cl::RPoint3D> edpoints;
			cl::Array<cl::RGB32Color> edcolors;
			//cout << "Loading Region...";
			const std::string pcfilename1 = dir + filename + "/" + str + "-region" + ".xyz";
			if (!cl::geometry::io::ReadXYZPoints(pcfilename1.c_str(), &allpoints, &allcolors))
			{
				cout << "Please check if " << pcfilename1 << " is exist." << endl;
				return -1;
			}
			//cout << "Successful." << endl;
			//cout << "Loading Edge...";
			const std::string pcfilename2 = dir + filename + "/Data/" + str + "-edge" + ".xyz";
			if (!cl::geometry::io::ReadXYZPoints(pcfilename2.c_str(), &edpoints, &edcolors))
			{
				cout << "Please check if " << pcfilename2 << " is exist." << endl;
				return -1;
			}
			//cout << "Successful." << endl;
			//cout << "Loading Target...";
			const std::string pcfilename = dir + filename + "/Data/" + str + "-r_target" + ".xyz";
			if (!cl::geometry::io::ReadXYZPoints(pcfilename.c_str(), &points, &colors))
			{
				cout << "Please check if " << pcfilename << " is exist." << endl;
				return -1;
			}
			//cout << "Successful." << endl;
			int dataslen = 0;
			string line;
			//cout << "Loading Feature...";
			ifstream data((dir + filename + "/Data/" + str + "-FPFH" + ".txt").c_str());
			for (int i = 0; i < points.size(); i++)
			{
				getline(data, line);
				stringstream ss;
				ss << line;
				for (int j = 0; j < d; j++)
				{
					ss >> datas[dataslen];
					dataslen++;
				}
			}
			//cout << "Successful." << endl;
			int **supervindices = new int*[points.size()];
			int *pnumofsuperv = new int[points.size()];
			ifstream dataofindices((dir + filename + "/Data/" + str + "-indices_target" + ".txt").c_str());
			for (int i = 0; i < points.size(); i++)
			{
				getline(dataofindices, line);
				stringstream ss;
				ss << line;
				ss >> pnumofsuperv[i];
				supervindices[i] = new int[pnumofsuperv[i]];
				for (int j = 0; j < pnumofsuperv[i]; ++j)
				{
					ss >> supervindices[i][j];
				}
			}
			//cout << "Loading ALL Successfully..." << endl;


			/*     Graph Generation    */

			static double settlink[100000][d];
			for (int i = 0; i < points.size(); i++)
			{
				for (int j = 0; j < d; j++)
				{
					settlink[i][j] = datas[i * d + j];
				}
			}
			vector<vector<int>> neighborsIndexes(points.size());
			vector<vector<float>> pointNKNSquaredDistance(points.size());
			int row, column;
			row = points.size();
			column = num_neigh;
			int *numNeighbors = new int[row];
			int ** neighborsIndexesp = new int*[row];
			double ** neighborsWeights = new double*[row];
			pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>); // Ä£ÐÍµãÔÆ
			model->width = points.size();
			model->height = 1;
			model->points.resize(model->width*model->height);
			for (int i = 0; i < points.size(); i++)
			{
				model->points[i].x = points[i].x;
				model->points[i].y = points[i].y;
				model->points[i].z = points[i].z;
			}
			//cout << "Kd-tree Building..." << endl;
			pcl::KdTreeFLANN<pcl::PointXYZ>kdtree1;
			kdtree1.setInputCloud(model);
			for (int i = 0; i < points.size(); i++)
			{
				double n_edgep = 0;
				neighborsIndexesp[i] = new int[num_neigh];
				neighborsWeights[i] = new double[num_neigh];
				numNeighbors[i] = num_neigh;
				if (kdtree1.nearestKSearch(model->points[i], num_neigh, neighborsIndexes[i], pointNKNSquaredDistance[i]) > 0)
				{
					//cout << endl << "Point " << i << " :";
					for (int j = 0; j < num_neigh; j++)
					{
						neighborsIndexesp[i][j] = neighborsIndexes[i][j];
						neighborsWeights[i][j] = smoothweight;
					}
					
					for (int j = 0; j < pnumofsuperv[i]; ++j)
					{
						if (edcolors[supervindices[i][j]] == red)
						{
							p_edgep[i] = 1;
							break;
						}
						p_edgep[i] = 0;
					}
				}
				//cout << p_edgep[i] << endl;
			}
			int backup[100000];

			/*   Iterative Energy Minimization   */

			for (int t = 0; t < iter; ++t)
			{
				int target, backg;
				target = backg = 0;
				cl::Array<int> slabels;
				//cl::Array<int> flabels;
				slabels.resize(points.size());
				//flabels.resize(fpoints.size());
				for (int i = 0; i < points.size(); i++)
				{
					if (colors[i] == red)
					{
						target++;
						slabels[i] = 1;
					}
					else
					{
						backg++;
						slabels[i] = 0;
					}
				}
				if (target == 0 || backg == 0)
				{
					cout << "Target " << str << " missed !!!" << endl;
					//test << "Target " << str << " missed !!!" << endl;
					//cout << "Output last result..." << endl;
					ofstream result((dir + filename + "/" + str + "-result" + ".xyz").c_str());
					for (int j = 0; j < points.size(); ++j)
					{
						if (backup[j] == 1)
						{
							for (int k = 0; k < pnumofsuperv[j]; ++k)
							{
								result << allpoints[supervindices[j][k]].x << " " << allpoints[supervindices[j][k]].y << " " << allpoints[supervindices[j][k]].z << " " << 255 << " " << 255 << " " << 255 << endl;
							}
						}
					}
					break;
				}
				int datatlen = 0;
				int datablen = 0;
				static double datat[100000 * d];
				static double datab[100000 * d];
				int tn, bn;
				tn = bn = 0;
				for (int i = 0; i < target + backg; i++)
				{
					if (slabels[i] == 1)
					{
						for (int j = 0; j < d; j++)
						{
							datat[datatlen] = datas[i * d + j];
							datatlen++;

						}
						tn++;
					}
					else
					{
						for (int j = 0; j < d; j++)
						{
							datab[datablen] = datas[i * d + j];
							datablen++;
						}
						bn++;
					}
				}
				//cout << "GMM Train " << t + 1 << endl;
				const int sizet = target; //Number of samples
				const int sizeb = backg;
				const int dim = d;   //Dimension of feature
				const int cluster_numt = 2; //Cluster number
				const int cluster_numb = 2;
				//auto t1 = chrono::steady_clock::now();
				GMM *gmmt = new GMM(dim, cluster_numt);//GMM has 2 SGM
				GMM *gmmb = new GMM(dim, cluster_numb);
				gmmt->Train(datat, sizet); //Training GMM
				gmmb->Train(datab, sizeb);
				//cout << "Train Complete..." << endl;
				static double tlinkt[100000];
				static double tlinkb[100000];
				double maxP = 0.0;
				for (int i = 0; i < target + backg; i++)
				{
					if (gmmt->GetProbability(settlink[i]) == 0.0) tlinkt[i] = 1000;
					else tlinkt[i] = -log(gmmt->GetProbability(settlink[i]));
				}
				for (int i = 0; i < target + backg; i++)
				{
					if (gmmb->GetProbability(settlink[i]) == 0.0) tlinkb[i] = 1000;
					else tlinkb[i] = -log(gmmb->GetProbability(settlink[i]));
				}

				delete gmmt;
				delete gmmb;

				GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(points.size(), 2);
				gc->setAllNeighbors(numNeighbors, neighborsIndexesp, neighborsWeights);
				for (int i = 0; i < points.size(); ++i)
				{
					gc->setDataCost(i, 1, tlinkt[i]);
					gc->setDataCost(i, 0, tlinkb[i]);
				}
				gc->setSmoothCost(&smoothFn);
				gc->setLabelCost(0.0);
				for (int i = 0; i < points.size(); i++)
				{
					gc->setLabel(i, slabels[i]);
				}
				for (int i = 0; i < points.size(); i++)
				{
					if (gc->whatLabel(i) == 1)
					{
						colors[i] = kcolors[0];
					}
					else if (gc->whatLabel(i) == 0)
					{
						colors[i] = kcolors[7];
					}
				}
				double before, after;
				gc->setLabelOrder(true);
				before = gc->compute_energy();

				gc->alpha_expansion(0);
				after = gc->compute_energy();
				if (before == after)
				{
					ofstream result((dir + filename + "/" + str + "-result" + ".xyz").c_str());
					for (int j = 0; j < points.size(); ++j)
					{
						if (gc->whatLabel(j) == 1)
						{
							backup[j] = 1;
							for (int k = 0; k < pnumofsuperv[j]; ++k)
							{
								result << allpoints[supervindices[j][k]].x << " " << allpoints[supervindices[j][k]].y << " " << allpoints[supervindices[j][k]].z << " " << 255 << " " << 255 << " " << 255 << endl;
							}
						}
						else
						{
							backup[j] = 0;
						}
					}
					break;
				}
				for (int i = 0; i < points.size(); i++)
				{
					if (gc->whatLabel(i) == 1)
					{
						colors[i] = kcolors[0];
					}
					else if (gc->whatLabel(i) == 0)
					{
						colors[i] = kcolors[7];
					}
				}
				delete[] gc;
			}
			points.clear();
			colors.clear();
			model->clear();
			for (int i = 0; i < points.size(); i++)
			{
				delete[] supervindices[i];
				delete[] neighborsIndexesp[i];
				delete[] neighborsWeights[i];
				vector<float>().swap(pointNKNSquaredDistance[i]);
				vector<int>().swap(neighborsIndexes[i]);
			}
			vector<vector<float>>().swap(pointNKNSquaredDistance);
			vector<vector<int>>().swap(neighborsIndexes);
			kdtree1.~KdTreeFLANN();
			delete[] pnumofsuperv;
			delete[] numNeighbors;
			delete[] neighborsIndexesp;
			delete[] neighborsWeights;
			//cout << ".";
		}
		cout << "Complete." << endl;

		/*    Evaluation    */

		int flag = 1;
		for (int i = 0; i < n; ++i)
		{
			double TP, TN, FP, FN;
			double F1, Re, Pre;
			TP = TN = FN = FP = 0.0;
			string str = std::to_string(i + 1);
			cl::Array<cl::RPoint3D> r_points;
			cl::Array<cl::RGB32Color> r_colors;
			cl::Array<cl::RPoint3D> allpoints;
			cl::Array<cl::RGB32Color> allcolors;
			cl::geometry::io::ReadXYZPoints((dir + filename + "/" + str + "-result" + ".xyz").c_str(), &r_points, &r_colors);
			cl::geometry::io::ReadXYZPoints((dir + filename + "/" + str + "-region" + ".xyz").c_str(), &allpoints, &allcolors);
			ifstream cloud((dir + filename + "/" + str + "-answers" + ".xyz").c_str());
			string pointdata;
			vector<vector<double>> a_points;
			cl::Array<cl::RGB32Color> allcolors1 = allcolors;
			cl::Array<cl::RGB32Color> allcolors2 = allcolors;
			while (getline(cloud, pointdata))
			{
				vector<double> point;
				double x, y, z;
				stringstream ss1;
				ss1 << pointdata;
				ss1 >> x;
				ss1 >> y;
				ss1 >> z;
				point.push_back(x);
				point.push_back(y);
				point.push_back(z);
				a_points.push_back(point);
				ss1.clear();
			}
			vector<int> points_a;
			vector<int> points_r;
			vector<int> pointsb_a;
			vector<int> pointsb_r;
			for (int j = 0; j < allpoints.size(); ++j)
			{
				if (allcolors[j] == red)
				{
					int f1, f2;
					f1 = f2 = 0;
					for (int jj = 0; jj < a_points.size(); ++jj)
					{
						if (allpoints[j].x == a_points[jj][0] && allpoints[j].y == a_points[jj][1] && allpoints[j].z == a_points[jj][2])
						{
							f1 = 1;
							break;
						}
					}
					if (f1 == 1)
					{
						points_a.push_back(1);
						pointsb_a.push_back(1);
					}
					else
					{
						points_a.push_back(0);
						pointsb_a.push_back(0);
					}

					for (int jj = 0; jj < r_points.size(); ++jj)
					{
						if (allpoints[j].x == r_points[jj].x&&allpoints[j].y == r_points[jj].y&&allpoints[j].z == r_points[jj].z)
						{
							f2 = 1;
							break;
						}
					}
					if (f2 == 1)
					{
						points_r.push_back(1);
						pointsb_r.push_back(1);
					}
					else
					{
						points_r.push_back(0);
						pointsb_r.push_back(0);
					}
				}
				else
				{
					pointsb_a.push_back(0);
					pointsb_r.push_back(0);
				}
			}
			//cout << points_a.size() << " " << points_r.size() << endl;
			for (int j = 0; j < allpoints.size(); ++j)
			{
				if (pointsb_a[j] == 0)
				{
					allcolors1[j] = white;
				}
				else
				{
					allcolors1[j] = red;
				}
				if (pointsb_r[j] == 0)
				{
					allcolors2[j] = white;
				}
				else
				{
					allcolors2[j] = red;
				}
			}
			if (cl::geometry::io::WriteXYZPoints((dir + "Benchmark/" + filename + "/" + str + "/" + str + ".xyz").c_str(), allpoints, allcolors1))
			{
				//cout << "The points are written into edge.xyz" << endl;
			}
			if (cl::geometry::io::WriteXYZPoints((dir + "Benchmark/" + filename + "/" + str + "/" + str + "-result.xyz").c_str(), allpoints, allcolors2))
			{
				//cout << "The points are written into edge.xyz" << endl;
			}
			for (int j = 0; j < points_a.size(); ++j)
			{
				if (points_a[j] == 1 && points_r[j] == 1)
				{
					TP++;
				}
				else if (points_a[j] == 0 && points_r[j] == 0)
				{
					TN++;
				}
				else if (points_a[j] == 1 && points_r[j] == 0)
				{
					FN++;
				}
				else if ((points_a[j] == 0 && points_r[j] == 1))
				{
					FP++;
				}
			}
			Re = TP / (TP + FN);
			Pre = TP / (TP + FP);
			F1 = 2 * Re * Pre / (Re + Pre);
			if (Re == 0)
			{
				n_t = n_t - 1;
				continue;
			}
			s_F1 = s_F1 + F1;
			s_Pre = s_Pre + Pre;
			s_Re = s_Re + Re;
			cout << str << setiosflags(ios::fixed) << setprecision(6) << ": F1=" << F1 << " Re=" << Re << " Pre=" << Pre << endl;
		}

	}
	cout << "Sweight=" << smoothweight << " " << "Eweight=" << edgeweight << " : ";
	cout << setiosflags(ios::fixed) << setprecision(6) << s_F1 / n_t << " " << s_Re / n_t << " " << s_Pre / n_t << endl;
	return 0;
}