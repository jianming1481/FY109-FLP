#ifndef _LASER_MAP_H_
#define _LASER_MAP_H_

#include <iostream> // cout, cerr
#include <vector>

using namespace std;

class LaserMap
{
public:
	LaserMap()
	{
	origin = vector<double>(3, 0.0);
	}
	~LaserMap(){}

	int negate;
	int height;
	int width;
	double resolution;
	double free_thresh;
	double occupied_thresh;
	string map_path;
	// Map Boundary
  	double max_x, max_y, min_x, min_y;
    vector<double> origin;
	vector<vector<double>> data;      // for storage the pixel value
};
#endif