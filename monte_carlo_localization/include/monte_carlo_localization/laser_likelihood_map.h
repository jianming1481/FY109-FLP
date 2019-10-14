#ifndef _LASER_LIKELIHOOD_MAP_H_
#define _LASER_LIKELIHOOD_MAP_H_

#include <iostream> // cout, cerr
#include <vector>
#include "laser_map.h"
using namespace std;

// static int likelihood_map_nums = 0;

class LaserLikelihoodMap
{
public:
	LaserLikelihoodMap(){}
	~LaserLikelihoodMap(){}
    LaserLikelihoodMap(LaserMap map)
    {
        height = map.height;
        width = map.width;
        resolution = map.resolution;
        origin = map.origin;
        // data = map.data;
    }

	int height;
	int width;
    
	double resolution;
    vector<double> origin;
	vector<vector<double>> data;      // for storage the pixel value
};
#endif