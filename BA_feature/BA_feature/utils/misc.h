#ifndef _MISC_H_
#define _MISC_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "types.h"

using namespace std;

inline vector<string> drwnGetLines(string path)
{
	vector<string> lines ; 

	ifstream fin(path.c_str());
	if(!fin.is_open()) 
	{
		cerr << "File not found " << path << endl;
		exit(-1);
	}
	while(!fin.eof())
	{
		char buffer[1024];
		fin.getline(buffer,1024);
		string name = buffer ; 
		if (name.compare("")==0)
		{
			break;
		}
		lines.push_back(name);
	}

	return lines ;
}

inline vector<CPoint> readTrainingData(string path)
{
	vector<CPoint> points ; 
	ifstream fin(path);
	if (!fin.is_open())
	{
		cout << "read file " << path << " error !" << endl ;
		exit(-1);
	}
	vector<string> lines = drwnGetLines(path);
	int num = lines.size(); 

	char buffer[1024];
	for (int i = 0 ; i < 80 ; i ++ )
	{
		fin.getline(buffer,1024);
	}

	for ( int i = 0 ; i < num - 80 ; i ++ )
	{
		CPoint point; 
		fin >> point.x >> point.y >> point.z >> point.label >> point.index ;
		points.push_back(point);
	}

	cout << "read points " << num << endl ;
	return points ;
}


inline vector<string> drwnReadFile(string filename)
{
	vector<string> names ; 

	ifstream fin(filename.c_str());
	if(!fin.is_open()) 
	{
		cerr << "File not found " << filename << endl;
		exit(-1);
	}
	while(!fin.eof())
	{
		string name ; 
		fin >> name ; 
		if (name.compare("")==0)
		{
			break;
		}
		names.push_back(name);
	}

	return names ;
}


#endif // !_UTILTS_H_