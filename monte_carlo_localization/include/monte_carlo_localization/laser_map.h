#ifndef _LASERMAP_H
#define _LASERMAP_H

#include <iostream> // cout, cerr
#include <vector>

using namespace std;

class LaserMap
{
public:
	LaserMap()
	{
	origin_ = vector<double>(3, 0.0);
	}
	~LaserMap(){}

	int negate_;
	int height_;
	int width_;
	double resolution_;
	double free_thresh_;
	double occupied_thresh_;

    vector<double> origin_;
	vector<vector<double>> data_;      // for storage the pixel value
};
#endif